
This is a rendered view generator support 3RScan dataset (potentially also ScanNet).

It assumes the "sequence.zip" is unziped with the same into the folder "sequence/" under the same directory.

The executable takes "--pth_in", the path to the "sequence/" folder.

The output is placed at the same folder.


# Build
```
mkdir build;
cd build;
cmake ..
make 
```

# Run
```
cd bin
./rio_renderer --pth_in /path/to/the/sequence/folder
```