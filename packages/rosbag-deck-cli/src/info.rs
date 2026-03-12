use std::path::Path;

use rosbag_deck::BagReader;
use rosbag_deck_ffi::Rosbag2Reader;

pub fn run(bag_path: &Path, storage_id: &str) -> anyhow::Result<()> {
    let reader = Rosbag2Reader::open(bag_path, storage_id)?;
    let meta = reader.metadata();

    let duration = meta.duration;
    let secs = duration.as_secs();
    let hours = secs / 3600;
    let mins = (secs % 3600) / 60;
    let sec = secs % 60;

    println!("Bag:      {}", bag_path.display());
    println!("Storage:  {}", meta.storage_identifier);
    println!("Duration: {:02}:{:02}:{:02}", hours, mins, sec);
    println!("Messages: {}", meta.message_count);
    println!();

    if meta.topics.is_empty() {
        println!("No topics.");
    } else {
        println!("{:<40} {:<40} {:>10}", "Topic", "Type", "Count");
        println!("{}", "-".repeat(92));
        for topic in &meta.topics {
            println!(
                "{:<40} {:<40} {:>10}",
                topic.name, topic.type_name, topic.message_count
            );
        }
    }

    Ok(())
}
