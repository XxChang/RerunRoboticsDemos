mod angle;

use nalgebra::{Matrix2, Rotation2, Vector2};
use rerun::{Color, LineStrip2D, LineStrips2D, RecordingStream, Vec2D};

pub fn plot_covariance_ellipse(rec: &RecordingStream, path: &str, x_c: f32, y_c: f32, cov: &Matrix2<f32>, chi: f32) {
    let eig = cov.symmetric_eigen();
    let a = (chi * eig.eigenvalues[0]).sqrt();
    let b = (chi * eig.eigenvalues[1]).sqrt();
    let angle = eig.eigenvectors.column(0)[1].atan2(eig.eigenvectors.column(0)[0]);
    plot_ellipse(rec, path, x_c, y_c, a, b, angle);
}

fn plot_ellipse(rec: &RecordingStream, path: &str, x_c: f32, y_c: f32, a: f32, b: f32, angle: f32) {
    let t: Vec<f32> = (0..)
        .map(|i| i as f32 * 0.1)
        .take_while(|&x| x <= 2.0 * std::f32::consts::PI)
        .collect();
    let x: Vec<f32> = t.iter().map(|t| a * t.cos()).collect();
    let y: Vec<f32> = t.iter().map(|t| b * t.sin()).collect();
    let rot = Rotation2::new(angle);
    let f: Vec<Vector2<f32>> = x.iter().zip(y.iter()).map(|(x, y)| rot * Vector2::new(*x, *y)).collect();
    let p: Vec<Vec2D> = f.iter().map(|v| Vec2D::new(x_c + v[0], y_c + v[1])).collect();
    let seg = vec![LineStrip2D::from_iter(p.iter())];
    rec.log(path, 
        &LineStrips2D::new(seg)
            .with_colors([Color::from_rgb(255, 0, 0)])
    ).unwrap();
}
