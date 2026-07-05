//! garage-push — push a locally-built firmware image to the ESP32 garage opener
//! over the LAN, and manage the device's auto-update pause flag.

use base64::Engine;
use clap::{Parser, Subcommand};
use std::time::{Duration, Instant};

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
    },
    /// Re-enable auto-updates (the GitHub self-poller).
    Resume {
        #[arg(long)]
        host: String,
        #[arg(long)]
        password: Option<String>,
    },
    /// Print the device status JSON.
    Status {
        #[arg(long)]
        host: String,
        #[arg(long)]
        password: Option<String>,
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
        Cmd::Push { host, file, password } => push(&host, &file, resolve_password(password)),
        Cmd::Resume { host, password } => resume(&host, resolve_password(password)),
        Cmd::Status { host, password } => status(&host, resolve_password(password)),
    };
    if let Err(e) = result {
        eprintln!("error: {e}");
        std::process::exit(1);
    }
}

fn push(host: &str, file: &str, pw: String) -> Result<(), String> {
    let data = std::fs::read(file).map_err(|e| format!("reading {file}: {e}"))?;
    let auth = auth_header(&pw);
    let url = format!("http://{host}/api/ota");
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
    wait_for_reboot(host, &auth)
}

fn resume(host: &str, pw: String) -> Result<(), String> {
    let auth = auth_header(&pw);
    let url = format!("http://{host}/api/auto-ota");
    ureq::post(&url)
        .set("Authorization", &auth)
        .send_string("on=1")
        .map_err(|e| format!("resume failed: {e}"))?;
    println!("Auto-updates re-enabled.");
    Ok(())
}

fn status(host: &str, pw: String) -> Result<(), String> {
    let auth = auth_header(&pw);
    let json = fetch_status(host, &auth)?;
    println!("{}", serde_json::to_string_pretty(&json).unwrap_or_else(|_| json.to_string()));
    Ok(())
}

fn fetch_status(host: &str, auth: &str) -> Result<serde_json::Value, String> {
    let url = format!("http://{host}/api/status");
    let body = ureq::get(&url)
        .set("Authorization", auth)
        .call()
        .map_err(|e| format!("status request failed: {e}"))?
        .into_string()
        .map_err(|e| format!("reading status body: {e}"))?;
    serde_json::from_str(&body).map_err(|e| format!("parsing status JSON: {e}"))
}

/// Poll /api/status until the device comes back up, then print its version.
fn wait_for_reboot(host: &str, auth: &str) -> Result<(), String> {
    // Give the device a moment to actually drop the connection and reboot.
    std::thread::sleep(Duration::from_secs(3));
    let deadline = Instant::now() + Duration::from_secs(90);
    while Instant::now() < deadline {
        if let Ok(json) = fetch_status(host, auth) {
            let ver = json.get("fw_version").and_then(|v| v.as_str()).unwrap_or("?");
            let paused = json
                .get("auto_ota_paused")
                .and_then(|v| v.as_bool())
                .unwrap_or(false);
            println!(
                "Device is up. Running firmware {ver}. Auto-updates: {}.",
                if paused { "PAUSED" } else { "on" }
            );
            println!("Run `garage-push resume --host {host}` to re-enable auto-updates.");
            return Ok(());
        }
        std::thread::sleep(Duration::from_secs(2));
    }
    Err("device did not come back online within 90s (check it manually)".into())
}
