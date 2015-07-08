# streamline-node

## Installing

`npm install`

## Loading Terrain Data

 * Install mongodb: `sudo apt-get install mongodb`
 * Download terrain data from the [SRTM data picker](http://srtm.csi.cgiar.org/SELECTION/inputCoord.asp), be sure to select the `ArcInfo ASCII` radio button.
    * Get at least tile [srtm_12_03](ftp://srtm.csi.cgiar.org/SRTM_v41/SRTM_Data_ArcASCII/srtm_13_03.zip), this is NW WA state and the test page expects this data to be available.
 * Unzip all of the datafiles in a common folder
 * Run `./bin/terrain_ingest [DATASET_PATH]`
    * You should see similar output to below
    * After output has stopped and all files reported `parsed` hit CTRL+C
 * Running the ingestion tool a second time will cause it to render all stored tiles into jpeg and png formats. This step is not required for WebGl rendering and is purely for debug. If you want this you will need to create all of the directories the program complains about not finding upon subsequent runs.
  
```
mongoose connected
ready
Rendering 0 tiles
parsing:/home/user/Datasets/ArcAscii/srtm_12_03/srtm_12_03.asc
tiling:/home/user/Datasets/ArcAscii/srtm_12_03/srtm_12_03.asc
parsed:/home/user/Datasets/ArcAscii/srtm_12_03/srtm_12_03.asc
```

## Running Service

`npm start`

The service runs on port 3000

## Test Pages

 * [terrain_test.html](localhost:3000/terrain_test.html)
 * [parser_test.html](localhost:3000/parser_test.html)

## TODO

 * Automate dataset downloading
 * Implement distributed ingestion process
 * Implement terrain tile service as a [hapi](http://hapijs.com/) micro-service
 * Implement frontend using component for [react](https://facebook.github.io/react/)
 * Implement raytracing on server side using three.js in server side canvas.
 * Ingest [peak meta-data](http://wiki.openstreetmap.org/wiki/Tag:natural%3Dpeak) from Open Street Maps
 * Screen scrape trail locations from [WTA trail finder](http://www.wta.org/go-hiking/map)