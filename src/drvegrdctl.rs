mod can;

use clap::Parser;
use can::{
    read_parameter, read_status, send_command, write_parameter, Command, Parameter, Status,
};
use log::debug;

#[derive(Parser, Debug, Clone)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// CAN device to use
    #[arg(short, long)]
    device: Option<String>,

    /// Monitor the CAN bus and print target lists.
    #[arg(short, long)]
    monitor: bool,

    /// Read the status from the device.
    #[arg(short, long)]
    status: bool,

    /// Command to send to the device
    #[arg(short, long, value_enum)]
    command: Option<Command>,

    /// Parameter to get or set
    #[arg(short, long, value_enum)]
    parameter: Option<Parameter>,

    /// Parameter value to set
    #[arg()]
    value: Option<u32>,
}

#[async_std::main]
async fn main() {
    env_logger::init();
    let args = Args::parse();

    let device = args.device.unwrap_or("can0".to_string());
    debug!("opening can interface {}", device);
    let sock = socketcan::async_io::CanSocket::open(&device).unwrap();

    if args.status {
        let software_generation = read_status(&sock, Status::SoftwareGeneration)
            .await
            .unwrap();
        let major_version = read_status(&sock, Status::MajorVersion).await.unwrap();
        let minor_version = read_status(&sock, Status::MinorVersion).await.unwrap();
        let patch_version = read_status(&sock, Status::PatchVersion).await.unwrap();
        let serial_number = read_status(&sock, Status::SerialNumber).await.unwrap();
        println!("Software Generation: {}", software_generation);
        println!(
            "Version: {}.{}.{}",
            major_version, minor_version, patch_version
        );
        println!("Serial Number: {}", serial_number);
    }

    if let Some(parameter) = args.parameter {
        if let Some(value) = args.value {
            let value = write_parameter(&sock, parameter, value).await.unwrap();
            println!("{:?}: {}", args.parameter, value);
        } else {
            let value = read_parameter(&sock, parameter).await.unwrap();
            println!("{:?}: {}", args.parameter, value);
        }
    }

    if let Some(command) = args.command {
        if let Some(value) = args.value {
            let value = send_command(&sock, command, value).await.unwrap();
            println!("{:?}: {}", args.command, value);
        } else {
            println!("Command {:?} requires a value", args.command);
            return;
        }
    }

    if args.monitor {
        loop {
            match can::read_message(&sock).await {
                Err(err) => println!("Error: {:?}", err),
                Ok(msg) => {
                    println!("{:?}", msg);
                }
            }
        }
    }
}
