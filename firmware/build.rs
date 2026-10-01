//! ESP-IDF build glue. `embuild` configures the include/link paths that the
//! `esp-idf-sys` bindings need.

fn main() {
    embuild::espidf::sysenv::output();
}
