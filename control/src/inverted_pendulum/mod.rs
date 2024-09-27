//! Inverted Pendulum Model
//! states: 

use nalgebra::{Matrix4, Vector1, Vector4};
use rerun::{Color, LineStrip2D, LineStrips2D, RecordingStream, Vec2D};

const BAR_LENGTH: f32 = 2.0;
const M: f32 = 1.0; // [kg]
const BODY_M: f32 = 0.1; // [kg]
const G: f32 = 9.8; // [m/s^2]
const DELTA_T: f32 = 0.1; // time tick [s]
const SIM_TIME: f32 = 5.0;

fn simulation(x: Vector4<f32>, u: Vector1<f32>) -> Vector4<f32> {
    let (a, b) = get_model_matrix();

    a * x + b * u
}

fn get_model_matrix() -> (Matrix4<f32>, Vector4<f32>) {
    let a = Matrix4::new(
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, BODY_M * G / M, 0.0,
        0.0, 0.0, 0.0, 1.0,
        0.0, 0.0, G * (M + BODY_M) / (M * BAR_LENGTH), 0.0
    );
    let a = Matrix4::identity() + a * DELTA_T;

    let b = Vector4::new(
        0.0, 1.0 / M, 0.0, 1.0 / (M * BAR_LENGTH)
    );

    (a, b)
}

pub fn plot_cart(rec: &RecordingStream, xt: f32, theta: f32) {
    let cart_w = 1.0;
    let cart_h = 0.5;
    let radius = 0.1;

    let cx = vec![
        -cart_w / 2.0, cart_w / 2.0, cart_w / 2.0, -cart_w / 2.0, -cart_w / 2.0, -cart_w / 2.0];
    let cy = vec![0.0, 0.0, cart_h, cart_h, 0.0];
    let cy = cy.iter().map(|v| v + radius * 2.0).collect::<Vec<f32>>();
    
    let cx = cx.iter().map(|v| v + xt).collect::<Vec<f32>>();

    let bx = vec![
        0.0, BAR_LENGTH * f32::sin(-theta)
    ];
    let bx = bx.iter().map(|v| v + xt).collect::<Vec<f32>>();
    let by = vec![
        cart_h, BAR_LENGTH * f32::cos(-theta) + cart_h
    ];
    let by = by.iter().map(|v| v + radius * 2.0).collect::<Vec<f32>>();

    let angles: Vec<f32> = (0..)
        .map(|i| i as f32 * 3.0f32.to_radians())
        .take_while(|&x| x <= 2.0 * std::f32::consts::PI)
        .collect();

    let ox = angles.iter().map(|&x| {
        radius * f32::cos(x)
    }).collect::<Vec<_>>();
    let oy = angles.iter().map(|&x| {
        radius * f32::sin(x)
    }).collect::<Vec<_>>();

    let rwx = ox.iter().map(|v| v + xt + cart_w / 4.0).collect::<Vec<f32>>();
    let rwy = oy.iter().map(|v| v + radius).collect::<Vec<f32>>();
    let lwx = ox.iter().map(|v| v + xt - cart_w / 4.0).collect::<Vec<f32>>();
    let lwy = oy.iter().map(|v| v + radius).collect::<Vec<f32>>();
    let wx = ox.iter().map(|v| v + bx.last().unwrap()).collect::<Vec<f32>>();
    let wy = oy.iter().map(|v| v + by.last().unwrap()).collect::<Vec<f32>>();

    let seg1 = cx.iter().zip(cy.iter()).map(|(x, y)| {
        Vec2D::new(*x, *y)
    }).collect::<Vec<_>>();
    let seg2 = bx.iter().zip(by.iter()).map(|(x, y)| {
        Vec2D::new(*x, *y)
    }).collect::<Vec<_>>();
    let seg3 = rwx.iter().zip(rwy.iter()).map(|(x, y)| {
        Vec2D::new(*x, *y)
    }).collect::<Vec<_>>();
    let seg4 = lwx.iter().zip(lwy.iter()).map(|(x, y)| {
        Vec2D::new(*x, *y)
    }).collect::<Vec<_>>();
    let seg5 = wx.iter().zip(wy.iter()).map(|(x, y)| {
        Vec2D::new(*x, *y)
    }).collect::<Vec<_>>();

    let seg = vec![
        LineStrip2D::from_iter(seg1.iter()),
        LineStrip2D::from_iter(seg2.iter()),
        LineStrip2D::from_iter(seg3.iter()),
        LineStrip2D::from_iter(seg4.iter()),
        LineStrip2D::from_iter(seg5.iter())
    ];

    rec.log(
        "control/inverted_pendulum/cart",
        &LineStrips2D::new(seg).with_colors([Color::from_rgb(0, 0, 255)])
    ).unwrap();
}

pub fn run(rec: &RecordingStream) {
    let mut x = Vector4::new(
        0.0, 0.0, 0.3, 0.0
    );

    let mut time = 0.0;
    while SIM_TIME > time {
        x = simulation(x, Vector1::new(0.2));
        plot_cart(rec, x[0], x[2]);
        time += DELTA_T;
    }
}