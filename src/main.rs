use async_std::task::block_on;
use deepviewrt::context::Context;
use drvegrd::{load_data, read_frame, Error, Packet};
use socketcan::{async_std::CanSocket, CanFrame, EmbeddedFrame, Id as CanId};
use std::time::Instant;

fn main() {
    let mut context = Context::new(None, 0, 0).unwrap();
    let model = std::fs::read("model.rtm").unwrap();
    context.load_model(model).unwrap();

    let mut input = context.tensor("batch_normalization_4/batchnorm/mul_1;batch_normalization_4/batchnorm/add_1;sequential_4/mlp_block/sequential/dense/MatMul;sequential_4/mlp_block/sequential/dense/Relu;sequential_4/mlp_block/sequential/dense/BiasAdd1_preact_dv").unwrap();
    let output = context.tensor("StatefulPartitionedCall:0").unwrap();

    println!("input: {} => {:?}", input.dims(), input.shape());
    println!("output: {} => {:?}", output.dims(), output.shape());

    let sock = CanSocket::open("can0").unwrap();

    loop {
        match read_frame(read_packet, &sock) {
            Err(err) => println!("Error: {:?}", err),
            Ok(frame) => {
                let mut labels = vec![false; frame.header.n_targets as usize];

                let now = Instant::now();
                match input.maprw_f32() {
                    Err(err) => panic!("failed to map input: {:?}", err),
                    Ok(mut map) => {
                        for idx in 0..frame.header.n_targets as usize {
                            map[idx * 5] = frame.targets[idx].speed as f32;
                            map[idx * 5 + 1] = frame.targets[idx].rcs as f32;
                            map[idx * 5 + 1] = frame.targets[idx].noise as f32;
                            map[idx * 5 + 1] = frame.targets[idx].power as f32;
                            map[idx * 5 + 1] = 0.0;
                        }
                    }
                }
                let input_time = now.elapsed();

                let now = Instant::now();
                context.run().unwrap();
                let model_time = now.elapsed();

                let now = Instant::now();
                match output.mapro_f32() {
                    Err(err) => panic!("failed to map output: {:?}", err),
                    Ok(map) => {
                        for idx in 0..frame.header.n_targets as usize {
                            labels[idx] = map[idx] > 0.5;
                        }
                    }
                }
                let output_time = now.elapsed();

                println!(
                    "[{}.{}] in: {:?} run: {:?} out: {:?} => {:?}",
                    frame.header.seconds,
                    frame.header.nanoseconds,
                    input_time,
                    model_time,
                    output_time,
                    labels.len(),
                );
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
