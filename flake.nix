{
  inputs = {
    flake-utils.url = "github:numtide/flake-utils";
    cargo-v5.url = "github:vexide/cargo-v5?ref=v0.12.1";
    rust-overlay.url = "github:oxalica/rust-overlay";
  };

  outputs =
    {
      nixpkgs,
      flake-utils,
      rust-overlay,
      cargo-v5,
      ...
    }:
    (flake-utils.lib.eachDefaultSystem (
      system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ (import rust-overlay) ];
        };
        cargo-v5' = cargo-v5.packages.${system}.default;
        rustToolchain = pkgs.rust-bin.fromRustupToolchainFile ./rust-toolchain.toml;
      in
      {
        devShell = pkgs.mkShell {
          buildInputs = (
            [
              pkgs.cargo-binutils
              (rustToolchain.override {
                extensions = [
                  "rust-analyzer"
                  "rust-src"
                  "clippy"
                ];
              })
            ]
            ++ (if system != "aarch64-darwin" then [ cargo-v5' ] else [ ])
          );
        };
      }
    ));
}
