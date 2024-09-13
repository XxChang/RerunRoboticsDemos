use nalgebra::{Matrix2, Matrix2x4, Matrix4, Matrix4x2, Vector2, Vector4};
use rand_distr::{Distribution, StandardNormal};
use rerun::{Color, LineStrip2D, LineStrips2D, Points2D, RecordingStream, Vec2D};

lazy_static::lazy_static! {
    static ref GPS_NOISE: Matrix2<f32> = Matrix2::from_diagonal(&Vector2::new(0.5_f32.powi(2), 0.5_f32.powi(2)));
    static ref INPUT_NOISE: Matrix2<f32> = Matrix2::from_diagonal(&Vector2::new(1.0f32.powi(2), 30.0f32.to_radians().powi(2)));
    static ref Q: Matrix4<f32> = Matrix4::from_diagonal(&Vector4::new(0.1_f32.powi(2), 0.1_f32.powi(2), 1.0f32.to_radians(), 1.0_f32.powi(2)));
    static ref R: Matrix2<f32> = Matrix2::from_diagonal(&Vector2::new(1.0_f32.powi(2), 1.0_f32.powi(2)));
}

const DT: f32 = 0.1;

fn calc_input() -> Vector2<f32> {
    // [m/s]
    let v = 1.0;
    // [rad/s]
    let yawrate = 0.1;

    Vector2::new(v, yawrate)
}

fn observation(ground_truth: &Vector4<f32>, dead_reckoning: &Vector4<f32>, u: &Vector2<f32>) -> (Vector4<f32>, Vector4<f32>, Vector2<f32>, Vector2<f32>) {
    let mut rng = rand::thread_rng();
    let normal_dist = StandardNormal;

    let x = motion_model(ground_truth, u);

    let z = observation_model(&x) + *GPS_NOISE * Vector2::new(normal_dist.sample(&mut rng), normal_dist.sample(&mut rng));

    let ud = u + *INPUT_NOISE * Vector2::new(normal_dist.sample(&mut rng), normal_dist.sample(&mut rng));

    let xd = motion_model(dead_reckoning, &ud);

    (x, xd, z, ud)
}

fn motion_model(x_prev: &Vector4<f32>, u: &Vector2<f32>) -> Vector4<f32> {
    let f = Matrix4::new(
        1.0, 0.0, 0.0, 0.0, 
        0.0, 1.0, 0.0, 0.0, 
        0.0, 0.0, 1.0, 0.0, 
        0.0, 0.0, 0.0, 0.0
    );

    let b = Matrix4x2::new(
        DT * f32::cos(x_prev[2]), 0.0,
        DT * f32::sin(x_prev[2]), 0.0, 
        0.0, DT, 
        1.0, 0.0
    );

    let x = f * x_prev + b * u;
    x
}

fn observation_model(x_after: &Vector4<f32>) -> Vector2<f32> {
    let h = Matrix2x4::new(
        1.0f32, 0.0, 0.0, 0.0, 
        0.0, 1.0, 0.0, 0.0
    );

    h*x_after
}

fn jacob_f(x: &Vector4<f32>, u: &Vector2<f32>) -> Matrix4<f32> {
    let yaw = x[2];
    let v = u[0];

    Matrix4::new(
        1.0, 0.0, -DT * v * yaw.sin(), DT * yaw.cos(), 
        0.0, 1.0, DT * v * yaw.cos(), DT * yaw.sin(), 
        0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0
    )
}

fn jacob_h() -> Matrix2x4<f32> {
    Matrix2x4::new(
        1.0, 0.0, 0.0, 0.0, 
        0.0, 1.0, 0.0, 0.0
    )
}

fn ekf_estimation(x_est: &Vector4<f32>, p_est: &Matrix4<f32>, z: &Vector2<f32>, u: &Vector2<f32>) -> (Vector4<f32>, Matrix4<f32>) {
    // Predict
    let x_pred = motion_model(x_est, u);
    let jf = jacob_f(x_est, u);
    let p_pred = jf * p_est * jf.transpose() + *Q;

    // Update
    let jh = jacob_h();
    let z_pred = observation_model(&x_pred);
    let res = z - z_pred;
    let s = jh * p_pred * jh.transpose() + *R;
    let k = p_pred * jh.transpose() * s.try_inverse().unwrap();
    let x_est = x_pred + k * res;
    let p_est = (Matrix4::identity() - k * jh) * p_pred;
    (x_est, p_est)
}

pub fn run(rec: &RecordingStream) {
    let mut x = Vector4::new(0.0, 0.0, 0.0, 0.0);
    let mut x_dr = Vector4::new(0.0f32, 0.0, 0.0, 0.0);
    let mut x_est = Vector4::new(0.0, 0.0, 0.0, 0.0);
    let mut p_est = Matrix4::<f32>::identity();

    let mut ground_truth = vec![Vec2D::new(x[0], x[1])];
    let mut observation_path = vec![Vec2D::new(x[0], x[1])];
    let mut dead_reckoning = vec![Vec2D::new(x[0], x[1])];
    let mut ekf_path = vec![Vec2D::new(x_est[0], x_est[1])];

    for i in 0..500 {
        let time = i as f32 * DT;

        rec.set_time_seconds("extended_kalman_filter", time as f64);

        let u = calc_input();

        let (x_true, xdr, z, ud) = observation(&x, &x_dr, &u);

        x = x_true;
        x_dr = xdr;

        let (xest, pest) = ekf_estimation(&x_est, &p_est, &z, &ud);

        x_est = xest;
        p_est = pest;

        ground_truth.push(Vec2D::new(x[0], x[1]));
        observation_path.push(Vec2D::new(z[0], z[1]));
        dead_reckoning.push(Vec2D::new(x_dr[0], x_dr[1]));
        ekf_path.push(Vec2D::new(x_est[0], x_est[1]));
        
        let seg = vec![LineStrip2D::from_iter(ground_truth.iter())];
        let seg_dr = vec![LineStrip2D::from_iter(dead_reckoning.iter())];
        let seg_ekf = vec![LineStrip2D::from_iter(ekf_path.iter())];

        rec.log(
            "localization/extended_kalman_filter/ground_truth", 
            &LineStrips2D::new(seg)
        ).unwrap();
        rec.log(
            "localization/extended_kalman_filter/observation",
            &Points2D::new(observation_path.iter().copied())
                .with_colors([Color::from_rgb(0, 255, 0)])
                .with_radii([0.02])
        ).unwrap();
        rec.log(
            "localization/extended_kalman_filter/dead_reckoning", 
            &LineStrips2D::new(seg_dr).with_colors([Color::from_rgb(10, 50, 100)])
        ).unwrap();
        rec.log(
            "localization/extended_kalman_filter/ekf_path", 
            &LineStrips2D::new(seg_ekf).with_colors([Color::from_rgb(255, 0, 0)])
        ).unwrap();
        utils::plot_covariance_ellipse(rec, "localization/extended_kalman_filter/pest", x_est[0], x_est[1], &p_est.fixed_view::<2,2>(0, 0).clone_owned(), 0.3)
    }
}
