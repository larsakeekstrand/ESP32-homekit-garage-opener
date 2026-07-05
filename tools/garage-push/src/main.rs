//! garage-push — push a locally-built firmware image to the ESP32 garage opener
//! over the LAN, and manage the device's auto-update pause flag.

use base64::Engine;
use clap::{Parser, Subcommand};
use std::time::{Duration, Instant};

/// Default web-server port. The device's HomeKit (HAP) server owns port 80, so the
/// diagnostics/config web server listens on 8080.
const DEFAULT_PORT: u16 = 8080;

#[derive(Parser)]
#[command(name = "garage-push", about = "Push firmware to the ESP32 garage opener over LAN")]
struct Cli {
    #[command(subcommand)]
    cmd: Cmd,
}

#[derive(Subcommand)]
enum Cmd {
    /// Upload a firmware image and reboot the device into it (pauses auto-updates).
    Push {
        #[arg(long)]
        host: String,
        #[arg(long)]
        file: String,
        #[arg(long)]
        password: Option<String>,
        #[arg(long, default_value_t = DEFAULT_PORT)]
        port: u16,
    },
    /// Re-enable auto-updates (the GitHub self-poller).
    Resume {
        #[arg(long)]
        host: String,
        #[arg(long)]
        password: Option<String>,
        #[arg(long, default_value_t = DEFAULT_PORT)]
        port: u16,
    },
    /// Print the device status JSON.
    Status {
        #[arg(long)]
        host: String,
        #[arg(long)]
        password: Option<String>,
        #[arg(long, default_value_t = DEFAULT_PORT)]
        port: u16,
    },
}

fn resolve_password(opt: Option<String>) -> String {
    if let Some(p) = opt {
        return p;
    }
    if let Ok(p) = std::env::var("GARAGE_PASSWORD") {
        return p;
    }
    rpassword::prompt_password("Device password: ").expect("failed to read password")
}

fn auth_header(pw: &str) -> String {
    let cred = base64::engine::general_purpose::STANDARD.encode(format!("admin:{pw}"));
    format!("Basic {cred}")
}

fn main() {
    let cli = Cli::parse();
    let result = match cli.cmd {
        Cmd::Push { host, file, password, port } => {
            push(&host, port, &file, resolve_password(password))
        }
        Cmd::Resume { host, password, port } => resume(&host, port, resolve_password(password)),
        Cmd::Status { host, password, port } => status(&host, port, resolve_password(password)),
    };
    if let Err(e) = result {
        eprintln!("error: {e}");
        std::process::exit(1);
    }
}

/// Turn a ureq error into a message that includes the device's response body
/// on an HTTP error status (not just the numeric code).
fn ureq_err(context: &str, e: ureq::Error) -> String {
    match e {
        ureq::Error::Status(code, resp) => {
            let body = resp.into_string().unwrap_or_default();
            format!("{context}: HTTP {code}: {body}")
        }
        e => format!("{context}: {e}"),
    }
}

fn push(host: &str, port: u16, file: &str, pw: String) -> Result<(), String> {
    let data = std::fs::read(file).map_err(|e| format!("reading {file}: {e}"))?;
    let auth = auth_header(&pw);
    let url = format!("http://{host}:{port}/api/ota");
    eprintln!("Uploading {} ({} bytes) to {url} ...", file, data.len());

    match ureq::post(&url)
        .set("Authorization", &auth)
        .set("Content-Type", "application/octet-stream")
        .send_bytes(&data)
    {
        Ok(resp) => {
            let body = resp.into_string().unwrap_or_default();
            eprintln!("Device accepted the image: {body}");
        }
        Err(ureq::Error::Status(code, resp)) => {
            let body = resp.into_string().unwrap_or_default();
            return Err(format!("device rejected upload (HTTP {code}): {body}"));
        }
        Err(e) => return Err(format!("upload failed: {e}")),
    }

    eprintln!("Waiting for the device to reboot into the new image ...");
    wait_for_reboot(host, port, &auth)
}

fn resume(host: &str, port: u16, pw: String) -> Result<(), String> {
    let auth = auth_header(&pw);
    let url = format!("http://{host}:{port}/api/auto-ota");
    ureq::post(&url)
        .set("Authorization", &auth)
        .send_string("on=1")
        .map_err(|e| ureq_err("resume failed", e))?;
    println!("Auto-updates re-enabled.");
    Ok(())
}

fn status(host: &str, port: u16, pw: String) -> Result<(), String> {
    let auth = auth_header(&pw);
    let json = fetch_status(host, port, &auth)?;
    println!("{}", serde_json::to_string_pretty(&json).unwrap_or_else(|_| json.to_string()));
    Ok(())
}

fn fetch_status(host: &str, port: u16, auth: &str) -> Result<serde_json::Value, String> {
    let url = format!("http://{host}:{port}/api/status");
    let body = ureq::get(&url)
        .set("Authorization", auth)
        .call()
        .map_err(|e| ureq_err("status request failed", e))?
        .into_string()
        .map_err(|e| format!("reading status body: {e}"))?;
    serde_json::from_str(&body).map_err(|e| format!("parsing status JSON: {e}"))
}

/// Poll /api/status until the device comes back up, then print its version.
fn wait_for_reboot(host: &str, port: u16, auth: &str) -> Result<(), String> {
    // Give the device a moment to actually drop the connection and reboot.
    std::thread::sleep(Duration::from_secs(3));
    let deadline = Instant::now() + Duration::from_secs(90);
    while Instant::now() < deadline {
        if let Ok(json) = fetch_status(host, port, auth) {
            let ver = json.get("fw_version").and_then(|v| v.as_str()).unwrap_or("?");
            let paused = json
                .get("auto_ota_paused")
                .and_then(|v| v.as_bool())
                .unwrap_or(false);
            println!(
                "Device is up. Running firmware {ver}. Auto-updates: {}.",
                if paused { "PAUSED" } else { "on" }
            );
            let port_arg = if port == DEFAULT_PORT { String::new() } else { format!(" --port {port}") };
            println!("Run `garage-push resume --host {host}{port_arg}` to re-enable auto-updates.");
            return Ok(());
        }
        std::thread::sleep(Duration::from_secs(2));
    }
    Err("device did not come back online within 90s (check it manually)".into())
}
