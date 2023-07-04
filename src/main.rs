use std::io::Read;
use std::time::Duration;

fn main() {
    let ports = serialport::available_ports().unwrap();
    if ports.is_empty() {
        return;
    }

    let port = &ports[0];
    let mut port = serialport::new(&port.port_name, 115200)
        .timeout(Duration::from_secs(1))
        .open()
        .expect("failed to open the port");

    println!("opened serial port: {}", port.name().unwrap());

    let mut serial_buf: Vec<u8> = vec![0; 1000];

    loop {
        match port.read(&mut serial_buf) {
            Ok(t) => {
                let s = match std::str::from_utf8(&serial_buf[..t]) {
                    Ok(v) => v,
                    Err(e) => panic!("Invalid UTF-8 sequence: {}", e),
                };

                println!("{}", s);
            }
            Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => (),
            Err(e) => eprintln!("{:?}", e),
        }
    }
}
