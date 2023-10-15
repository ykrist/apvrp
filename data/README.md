# Instances

Each instance is identified with an integer index as well as its name.  The instances used in the paper are as follows:

| Instances | Indices      | Archive File               | Filenames in archive |
|-----------|--------------|----------------------------|----------------------|
| A         | `[0, 79]`    | `apvrp_tilk.tar.xz`        | `A??TW*.txt`         |
| B         | `[190, 269]` | `apvrp_tilk_b_part.tar.xz` | `B??TW*_3p.txt`      |
| MK        | `[160, 189]` | `apvrp_meisel.tar.xz`      | `A??.txt`            |

You can use the `dinfo` binary to print information about an instance using its index.

```bash
dinfo INDEX
# or run using cargo without installing:
cargo run --bin dinfo INDEX
```

The APVRP code will automatically extract these archives into `data/uncompressed`.

The format of these files is left unchanged from original files created by Meisel and Kopfer.
