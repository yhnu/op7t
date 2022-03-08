extern crate clap;
use clap::{App, Arg};
use regex::Regex;
use regex::RegexBuilder;
use std::collections::HashMap;
use std::fs::File;
use std::io::{BufRead, BufReader, Read};
use std::path::Path;
use std::process;
use std::process::Command;

struct map {
    pub start: u64,
    pub end: u64,
    pub filename: String,
}

impl map {
    pub fn new(start: u64, end: u64, filename: String) -> Self {
        return map {
            start,
            end,
            filename,
        };
    }
}

fn parser_krhook_log(krlog: &Path, sym: &str, addr2line: &str) {
    let file = File::open(krlog).unwrap();
    let reader = BufReader::new(file);

    let re = r"0x(?P<start>[0-9a-f]*)-0x(?P<end>[0-9a-f]*) 0x(?P<pgoff>[0-9a-f]*)";
    let re: Regex = RegexBuilder::new(re).multi_line(true).build().unwrap();

    let re2 = r"0x([0-9a-f]*)";
    let re2: Regex = RegexBuilder::new(re2).multi_line(true).build().unwrap();

    let mut map_base: HashMap<String, u64> = HashMap::new();
    let mut map_list = vec![];
    let mut open_path = "".to_string();
    for line in reader.lines() {
        let line = line.unwrap();
        let line = line.as_str();
        let mut elements: Vec<&str> = line.split(' ').collect();
        if line.contains("[s]openat ") {
            open_path = elements.pop().unwrap().to_string();
            // println!("{}", open_path);
        } else if line.contains("[m]0x") {
            let last_1 = elements.pop().unwrap();
            let last_2 = elements.pop().unwrap();
            let cap = re.captures(line);
            match cap {
                Some(cap) => {
                    if last_1.ends_with(".so") {
                        let start = u64::from_str_radix(&cap["start"], 16).unwrap();
                        let end = u64::from_str_radix(&cap["end"], 16).unwrap();
                        let pgoff = u64::from_str_radix(&cap["pgoff"], 16).unwrap();                        
                        // println!("{:#x} {:#x} {:#x}", start, end, pgoff);
                        if last_2.contains("x") {
                            let real_start = start - pgoff;
                            map_list.push(map::new(real_start, end, last_1.to_string()));
                            if map_base.contains_key(last_1) {
                                if map_base[last_1] > real_start {
                                    if let Some(e) = map_base.get_mut(last_1) {
                                        *e = real_start;
                                    }
                                }
                            } else {
                                map_base.insert(last_1.to_string(), real_start);
                            }
                        }
                    }
                }
                None => { //println!("[err] {}", line),
                }
            }
        } else if line.contains("[e]openat ") {            
            println!("-----------------------------------------");
            println!("openat[{}] dump stack:", open_path);
            let stack = elements.pop().unwrap();
            if stack.contains("|") {                
                for cap in re2.captures_iter(stack) {
                    // println!("{:?}", &cap);
                    let pc = u64::from_str_radix(&cap[1], 16).unwrap();                    
                    for e in map_list.iter() {                        
                        let base = map_base[&e.filename];
                        let offset = pc - base;
                        if pc >= e.start && pc <= e.end {                            
                            let dir = Path::new(sym);                                                        
                            if dir.exists() {                                
                                let so_path = Path::new(&e.filename).file_name().unwrap().to_str().unwrap();
                                let so_path = format!("{}/{}", sym, so_path);
                                if Path::new(&so_path).exists() && Path::new(addr2line).exists() {
                                    // println!("{}", so_path);
                                    // let args = format!("{} -f -e {} {:#x}", addr2line, so_path, pc);
                                    // println!("{}", args);
                                    let out = Command::new(addr2line)
                                    .arg("-f")
                                    .arg("-e")
                                    .arg(so_path)
                                    .arg(format!("{:#x}", offset))
                                    .output()
                                    .unwrap();
                                    // println!("{} {} -f -e {:?}", addr2line, so_path, out.stdout);                                    
                                    let out = String::from_utf8(out.stdout).unwrap().replace("\r\n", " ");
                                    println!("{}/{:#x} + {:#x} ({})", e.filename, base, offset, out);
                                } else {
                                    println!("{}/{:#x} + {:#x}", e.filename, base, offset);
                                }                                                        
                            } else {
                                println!("{}/{:#x} + {:#x}", e.filename, base, offset);
                            }
                        }
                    }                    
                }
            }

            // for (name, base) in map_base.iter() {
            //     println!("{} {:#x}", name, base);
            // }

            map_list.clear();
            open_path = "".to_string();
        }
    }
}
fn main() {
    let matches = App::new("krhook offline stack walk")
        .version("1.0")
        .author("yiluoyang <buutuud@gmail.com>")
        .after_help("EXAMPLE:\n    kr_offline demo.krhook.txt sym")
        .arg(Arg::with_name("kr").help("kr log file").required(true))
        .arg(Arg::with_name("sym").help("sym dir").required(false))
        .get_matches();

    let kr = matches.value_of("kr").unwrap();
    let kr = Path::new(kr);
    if !kr.exists() {
        println!("kr file({}) is not exists", kr.display());
        process::exit(-1);
    }

    let sym = matches.value_of("sym").unwrap_or_default();
    // let sym = Path::new(sym);
    // if !sym.exists() {
    //     println!("sym file({}) is not exists", sym.display());
    //     process::exit(-1);
    // }

    let addr2line = r"D:\Android\android-ndk-r20\toolchains\llvm\prebuilt\windows-x86_64\bin\aarch64-linux-android-addr2line.exe";

    // let symbol_file = parse_breakpad_symbol_file(input);
    // parser_ips(ips, "UnityFramework", &symbol_file);
    parser_krhook_log(kr, sym, addr2line);
    process::exit(0);
}
