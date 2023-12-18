use async_std::task::block_on;
use clap::Parser;
use deepviewrt::context::Context;
use drvegrd::{load_data, read_frame, Error, Packet};
use rerun::external::re_sdk_comms::DEFAULT_SERVER_PORT;
use rerun::{Color, Points3D, RecordingStream, RecordingStreamResult, TimeSeriesScalar};
use socketcan::{async_std::CanSocket, CanFrame, EmbeddedFrame, Id as CanId};
use std::net::{Ipv4Addr, SocketAddr};
use std::time::Instant;
use std::f32::consts::PI;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// connect to remote rerun viewer at this address
    #[arg(short, long)]
    connect: Option<Ipv4Addr>,

    /// use this port for the rerun viewer (remote or web server)
    port: Option<u16>,

    /// zenoh connection mode
    #[arg(short, long, default_value = "peer")]
    mode: String,

    /// connect to endpoint
    #[arg(short, long)]
    endpoint: Vec<String>,

    /// ros topic
    #[arg(short, long, default_value = "rt/camera/compressed")]
    topic: String,
}

#[async_std::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();

    let rr = if let Some(addr) = args.connect {
        let port = args.port.unwrap_or(DEFAULT_SERVER_PORT);
        let remote = SocketAddr::new(addr.into(), port);
        rerun::RecordingStreamBuilder::new("radar-people")
            .connect_opts(remote, rerun::default_flush_timeout())?
    } else {
        rerun::RecordingStreamBuilder::new("radar-people").spawn()?
    };

    rerun::Logger::new(rr.clone())
        .with_path_prefix("logs")
        .with_filter(rerun::default_log_filter())
        .init()?;
    log::info!("starting radar people classifier");
    log::logger().flush();

    let mut context = Context::new(None, 0, 0)?;
    let model = std::fs::read("model.rtm")?;
    context.load_model(model)?;

    let mut input = context.tensor("serving_default_input_101:0")?;
    let output = context.tensor("StatefulPartitionedCall:0")?;

    println!("input: {} => {:?}", input.dims(), input.shape());
    println!("output: {} => {:?}", output.dims(), output.shape());

    let sock = CanSocket::open("can0")?;

    loop {
        match read_frame(read_packet, &sock) {
            Err(err) => println!("Error: {:?}", err),
            Ok(frame) => {
                let now = Instant::now();
                match input.maprw_f32() {
                    Err(err) => panic!("failed to map input: {:?}", err),
                    Ok(mut map) => {
                        for idx in 0..frame.header.n_targets {
                            let tgt = &frame.targets[idx];
                            map[idx * 4] = tgt.speed as f32;
                            map[idx * 4 + 1] = tgt.power as f32;
                            map[idx * 4 + 2] = tgt.rcs as f32;
                            map[idx * 4 + 3] = tgt.noise as f32;
                        }
                    }
                }

                context.run()?;
                log_time(&rr, "model", now)?;

                let now = Instant::now();
                let mut labels = vec![0.0f32; frame.header.n_targets];
                match output.mapro_f32() {
                    Err(err) => panic!("failed to map output: {:?}", err),
                    Ok(map) => {
                        for idx in 0..frame.header.n_targets {
                            labels[idx] = map[idx];
                        }
                    }
                }

                rr.log(
                    "radar",
                    &Points3D::new((0..frame.header.n_targets).map(|idx| {
                        let tgt = &frame.targets[idx];
                        rae2xyz(tgt.range as f32, tgt.azimuth as f32, tgt.elevation as f32)
                    }))
                    .with_radii([0.5])
                    .with_labels(
                        (0..frame.header.n_targets)
                            .map(|idx| format!("{} => {:?}", labels[idx], frame.targets[idx])),
                    )
                    .with_colors(labels.iter().map(|l| {
                        if *l > 0.5 {
                            Color::from_rgb(0, 255, 0)
                        } else {
                            Color::from_rgb(255, 255, 255)
                        }
                    })),
                )?;

                log_time(&rr, "draw", now)?;
            }
        }
    }
}

fn read_packet(can: &CanSocket) -> Result<Packet, Error> {
    match block_on(can.read_frame()) {
        Ok(CanFrame::Data(frame)) => {
            let id = match frame.id() {
                CanId::Standard(id) => id.as_raw() as u32,
                CanId::Extended(id) => id.as_raw(),
            };
            Ok(Packet {
                id,
                data: load_data(frame.data()),
            })
        }
        Ok(CanFrame::Remote(frame)) => panic!("Unexpected remote frame: {:?}", frame),
        Ok(CanFrame::Error(frame)) => panic!("Unexpected error frame: {:?}", frame),
        Err(err) => Err(Error::Io(err)),
    }
}

fn log_time(rr: &RecordingStream, name: &str, t: Instant) -> RecordingStreamResult<()> {
    rr.log(
        name,
        &TimeSeriesScalar::new(t.elapsed().as_nanos() as f64 / 1e9),
    )
}

fn rae2xyz(range: f32, azimuth: f32, elevation: f32) -> [f32; 3] {
    let azi = azimuth / 180.0 * PI;
    let ele = elevation /  180.0 * PI;
    let x = range * ele.cos() * azi.cos();
    let y = range * ele.cos() * azi.sin();
    let z = range * ele.sin();
    [x, y, z]
}
