{
  inputs.nixpkgs.url = "github:nixos/nixpkgs/nixos-22.11";
  outputs = { self, nixpkgs }:
  let
    pkgs = import nixpkgs { config.allowUnfree = true; system = "x86_64-linux"; };
  in
  {
    devShells.x86_64-linux.default = pkgs.mkShell {
      buildInputs = with pkgs; [ python3 vscode-fhs git ];
    };
  };
}
