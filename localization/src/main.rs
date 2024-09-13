mod extended_kalman_filter;

fn main() {
    let rec = rerun::RecordingStreamBuilder::new("localization")
        .connect().unwrap();
    extended_kalman_filter::extended_kalman_filter::run(&rec);
}
