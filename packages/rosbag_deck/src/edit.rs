use std::{collections::HashSet, path::PathBuf};

use regex::Regex;
use rosbag_deck_core::{BagReader, EditConfig, EditPipeline};
use rosbag_deck_ffi::{Rosbag2Reader, Rosbag2Writer};

pub struct EditOpts {
    pub input: Vec<PathBuf>,
    pub output: Option<PathBuf>,
    pub format: String,
    pub storage: String,
    pub topics: Option<Vec<String>>,
    pub regex: Option<String>,
    pub exclude: Option<String>,
    pub start: Option<f64>,
    pub end: Option<f64>,
    pub duration: Option<f64>,
    pub time_offset: Option<f64>,
    pub time_scale: f64,
    pub dry_run: bool,
    pub verbose: bool,
}

pub fn run(opts: &EditOpts) -> anyhow::Result<()> {
    // Validate: output is required unless --dry-run.
    if opts.output.is_none() && !opts.dry_run {
        anyhow::bail!("--output is required (or use --dry-run)");
    }

    // Open all input bags.
    let mut readers: Vec<Box<dyn BagReader>> = Vec::new();
    for path in &opts.input {
        let reader = Rosbag2Reader::open(path, &opts.storage)?;
        eprintln!(
            "  Input: {} ({}, {} messages)",
            path.display(),
            reader.metadata().storage_identifier,
            reader.metadata().message_count,
        );
        readers.push(Box::new(reader));
    }

    // Compute merged time range for relative start/end.
    let global_start_ns = readers
        .iter()
        .map(|r| r.metadata().start_time_ns)
        .min()
        .unwrap_or(0);
    let global_end_ns = readers
        .iter()
        .map(|r| r.metadata().end_time_ns)
        .max()
        .unwrap_or(0);

    // Resolve topic filter.
    let all_topic_names: Vec<String> = {
        let mut names = HashSet::new();
        for reader in &readers {
            for topic in &reader.metadata().topics {
                names.insert(topic.name.clone());
            }
        }
        names.into_iter().collect()
    };

    let topic_filter = resolve_topic_filter(
        &all_topic_names,
        opts.topics.as_deref(),
        opts.regex.as_deref(),
        opts.exclude.as_deref(),
    )?;

    // Resolve time range.
    let time_range = resolve_time_range(
        global_start_ns,
        global_end_ns,
        opts.start,
        opts.end,
        opts.duration,
    );

    // Open writer (unless dry run).
    let writer = if let Some(ref output) = opts.output {
        if !opts.dry_run {
            Some(Box::new(Rosbag2Writer::open(output, &opts.format)?)
                as Box<dyn rosbag_deck_core::writer::BagWriter>)
        } else {
            None
        }
    } else {
        None
    };

    let time_offset_ns = opts.time_offset.map(|s| (s * 1e9) as i64).unwrap_or(0);

    let config = EditConfig {
        topic_filter,
        time_range,
        time_offset_ns,
        time_scale: opts.time_scale,
        dry_run: opts.dry_run,
        verbose: opts.verbose,
    };

    // Print summary before running.
    eprintln!();
    if opts.dry_run {
        eprintln!("  Dry run mode — no output will be written.");
    }
    if let Some(ref output) = opts.output {
        eprintln!("  Output: {} (format: {})", output.display(), opts.format);
    }
    if let Some((start_ns, end_ns)) = time_range {
        let start_s = (start_ns - global_start_ns) as f64 / 1e9;
        let end_s = (end_ns - global_start_ns) as f64 / 1e9;
        eprintln!("  Time range: {start_s:.3}s - {end_s:.3}s");
    }
    if opts.time_offset.is_some() || (opts.time_scale - 1.0).abs() > f64::EPSILON {
        eprintln!(
            "  Transform: scale={:.3}x, offset={:.3}s",
            opts.time_scale,
            opts.time_offset.unwrap_or(0.0),
        );
    }
    eprintln!();

    let pipeline = EditPipeline::new(readers, writer, config);
    let stats = pipeline.run()?;

    // Print results.
    eprintln!();
    eprintln!("  Done in {:.2}s", stats.elapsed.as_secs_f64());
    eprintln!(
        "  Messages: {} read, {} written, {} filtered",
        stats.messages_read, stats.messages_written, stats.messages_filtered,
    );
    if stats.messages_written > 0 {
        let dur_s = (stats.output_time_range.1 - stats.output_time_range.0) as f64 / 1e9;
        eprintln!("  Output duration: {dur_s:.3}s");
        eprintln!("  Topics written:");
        let mut topics: Vec<&String> = stats.topics_written.iter().collect();
        topics.sort();
        for topic in topics {
            eprintln!("    {topic}");
        }
    }
    let throughput = stats.messages_read as f64 / stats.elapsed.as_secs_f64().max(0.001);
    eprintln!("  Throughput: {throughput:.0} msgs/sec");

    Ok(())
}

fn resolve_topic_filter(
    all_topics: &[String],
    topics: Option<&[String]>,
    regex_pattern: Option<&str>,
    exclude_pattern: Option<&str>,
) -> anyhow::Result<Option<HashSet<String>>> {
    let mut accepted: HashSet<String> = if let Some(topics) = topics {
        topics.iter().cloned().collect()
    } else if regex_pattern.is_some() || exclude_pattern.is_some() {
        all_topics.iter().cloned().collect()
    } else {
        return Ok(None);
    };

    if let Some(pattern) = regex_pattern {
        let re =
            Regex::new(pattern).map_err(|e| anyhow::anyhow!("invalid --regex pattern: {e}"))?;
        for topic in all_topics {
            if re.is_match(topic) {
                accepted.insert(topic.clone());
            }
        }
    }

    if let Some(pattern) = exclude_pattern {
        let re =
            Regex::new(pattern).map_err(|e| anyhow::anyhow!("invalid --exclude pattern: {e}"))?;
        accepted.retain(|topic| !re.is_match(topic));
    }

    Ok(Some(accepted))
}

/// Resolve relative start/end/duration into absolute nanosecond time range.
fn resolve_time_range(
    global_start_ns: i64,
    global_end_ns: i64,
    start_secs: Option<f64>,
    end_secs: Option<f64>,
    duration_secs: Option<f64>,
) -> Option<(i64, i64)> {
    let has_any = start_secs.is_some() || end_secs.is_some() || duration_secs.is_some();
    if !has_any {
        return None;
    }

    let start_ns = match start_secs {
        Some(s) => global_start_ns + (s * 1e9) as i64,
        None => global_start_ns,
    };

    let end_ns = match (end_secs, duration_secs) {
        (Some(e), _) => global_start_ns + (e * 1e9) as i64,
        (None, Some(d)) => start_ns + (d * 1e9) as i64,
        (None, None) => global_end_ns,
    };

    Some((start_ns, end_ns))
}
