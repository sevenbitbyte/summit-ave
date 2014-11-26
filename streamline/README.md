Streamline README
===

This is a prototype for the Streamline cumulo application. This version only supports displaying location data and uses a the UTM transform to render data. In the future UTM will not be used and data will be stored into a database rather than read from files at each run. In the future topographic renders will be used as the background.

Demo
--
A video demo of how to run Streamline can be found [here](http://cumulonimbus.com/175f4f1b3f8625b0d18796c9ef2f2371/)

[Roadmap](roadmap.md)
---

Building
---
This project is QMake based:

	mkdir -p build
	cd build/
	cmake ../
	make


Running
---

	cd build/
	./bin/streamline

Controls
---
* WASD - Move x,y
* Arrows Keys - Pan and tilt
* +/- - Zoom in/out
* Left click and drag - move x,y
* Right click and drag - pan and tilt
* Mouse wheel - Zoom

Collecting GPX Data
---
* Android
    * [GPX Logger](https://play.google.com/store/apps/details?id=com.mendhak.gpslogger&hl=en)
* iOS
    * [myTracks](https://itunes.apple.com/us/app/mytracks-the-gps-logger/id358697908?mt=8)
* Windows Phone
    * [GPS & GPX Logger](http://www.windowsphone.com/en-us/store/app/gps-gpx-logger/3e8d4a3a-0ff6-df11-9264-00237de2db9e)

GPX Datasets
---
Request access from [Alan Meekins](mailto:alan.meekins@gmail.com)

    git clone git@sroz.net:nullagent_datasets.git
    
Topo Rendering
---

<img src="http://cumulonimbus.com/175f4f1b3f8625b0d18796c9ef2f2371/topo/centralVAcrop.png" width=100%>


####Examples####

* Western Washington([ref](https://maps.google.com/?ll=46.890232,-121.970215&spn=4.24188,6.778564&t=p&z=7))
    * [black and white](http://cumulonimbus.com/175f4f1b3f8625b0d18796c9ef2f2371/topo/srtm_12_03.asc.inv.bw.jpg)
    * [HSV lowpass](http://cumulonimbus.com/175f4f1b3f8625b0d18796c9ef2f2371/topo/srtm_12_03.asc.bandpass600-hsv.jpg)
    
* Central VA([ref](https://maps.google.com/?ll=37.844495,-78.678589&spn=2.450731,3.389282&t=p&z=8))
    * [black and white](http://cumulonimbus.com/175f4f1b3f8625b0d18796c9ef2f2371/topo/srtm_21_05.asc.inv.bw.jpg)
    * [HSV bandpass](http://cumulonimbus.com/175f4f1b3f8625b0d18796c9ef2f2371/topo/srtm_21_05.asc.bandpass600-hsv.jpg)
