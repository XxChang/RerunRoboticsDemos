mod inverted_pendulum;

fn main() {
    let rec = rerun::RecordingStreamBuilder::new("control")
        .connect().unwrap();
    inverted_pendulum::run(&rec);
    println!("Hello, world!");
}
