The following proposed feature set is grouped by affinity and shows priority (1 is highest, 10 is lowest.)

Many images are from commercial packages.  We do not intend to duplicate their GUI; these images are for discussion purposes only.

## Menu ##
| **Feature** | **Priority** | **Release** | **Comment** |
|:------------|:-------------|:------------|:------------|
| Hotkey support | 3 | initial |  |
| Toolbar | 1 | initial |  |

## Colors ##
| **Feature** | **Priority** | **Release** | **Comment** |
|:------------|:-------------|:------------|:------------|
| Color options: NOAA / S-57 | 4 |  |  |
| Day / dusk / night palettes | 4 |  |  |

One way to implement this would be to drive the display from a selection of XML definitions for layers, which would specify line color, area fill color, line weight and friendly layer name, and max/min zoom level for visibility.  Some layers contain details that should not be visible at low magnification (on a small scale chart.)  The XML file that defines layer appearance then we would be responsible for the contents of the XML file.  For example:
```
<layers lineColor=”#000000” fillColor=”#00ffff” visible=”no”>
    <layer shortName=”LNDARE” friendlyName=”Land”
           lineColor=”#97935d” fillColor=”#cac57d”
           minZoom=”40000” maxZoom=”200000” />
</layers>
```

  1. Default values for unspecified layer colors would be specified by the `layers` attributes;
  1. The visible attribute would only apply to layers not specified by `layer` elements; in other words, the visible attribute of the layers element indicates if layers should be displayed that are not described by any layer element;
  1. Zoom levels would be interpreted as 1:X, where X is specified in the XML file. For example, 1:50,000 would be specified in XML simply as 50000.  The layer should be displayed if the zoom level is between `minZoom` to `maxZoom`, inclusive;
  1. If `minZoom` is not specified or has the value zero, then the layer should be always displayed for zoom levels below `maxZoom`;
  1. If `maxZoom` is not specified or has the value zero, then the layer should be always displayed for zoom levels above `minZoom`;
  1. If a layer occurs in the data file that is not defined in the XML file, then its appearance would revert to default values (`friendlyName` same as `shortName`, outline color black, fill color light grey.)

## Chart support ##
| **Feature** | **Priority** | **Release** | **Comment** |
|:------------|:-------------|:------------|:------------|
| S-57 v2 (DX90) | 1 | initial |  |
| S-57 v3.1 | 5 |  |  |
| S-63 | 4 |  |  |
| BSB-4 | 2 |  |  |
| CMAP CM93/3 | 6 |  |  |
| ARCS (Admiralty Raster Chart Service) | 6 |  | raster format; available for almost any region of the world ocean |
| User input | 6 |  |  |
| Toggle display of chart boundaries | 6 |  |  |
| Chart folder specification | 6 |  |  |
| Autoinstall charts when a CD or DVD is inserted | 6 |  |  |

## Viewport Control ##
| Duplicate viewport (Control/Command N) | 1 | | |
|:---------------------------------------|:--|:|:|
| Close viewport (Control/Command W) | 1 |  |  |
| Zoom window (pick 2 points with a mouse) | 1 |  |  |
| Zoom in /out (plus and minus keys) | 1 |  |  |
| Automatically select appropriate chart(s) for given display area and magnification (this selection is per viewport) | 1 |  |  If a detail chart is available and appropriate for the current zoom level, show it, however if the detail chart does not cover the entire viewport then also load and display larger scale chart(s) for the area not covered by the detail chart.  This implies a need for an index of chart extents and scale factors.  Flush charts that are no longer required for the current zoom level |
| Panning via arrow keybs and click-drag by mouse | 6 |  |  |
| Clicking on a location should center about that point and zoom the chart by 100% | 6 |  |  |

![http://marnav.googlecode.com/svn/trunk/docs/images/charts.png](http://marnav.googlecode.com/svn/trunk/docs/images/charts.png)

## Overlays ##
| **Feature** | **Priority** | **Release** | **Comment** |
|:------------|:-------------|:------------|:------------|
| GRB | 2 |  | read from a file by a plug-in |
| AIS | 2 |  | live data supplied from a plug-in |
| Radar | 2 |  | live data supplied from a plug-in |
| DSC | 2 |  | live data supplied from a plug-in |

### GRB ###
![http://marnav.googlecode.com/svn/trunk/docs/images/gribSetup.png](http://marnav.googlecode.com/svn/trunk/docs/images/gribSetup.png)
![http://marnav.googlecode.com/svn/trunk/docs/images/gribLayers.png](http://marnav.googlecode.com/svn/trunk/docs/images/gribLayers.png)

With only Wind and pressure layers enabled:
![http://marnav.googlecode.com/svn/trunk/docs/images/grib.png](http://marnav.googlecode.com/svn/trunk/docs/images/grib.png)

With all layers enabled:
![http://marnav.googlecode.com/svn/trunk/docs/images/grib2.png](http://marnav.googlecode.com/svn/trunk/docs/images/grib2.png)

## Export Formats ##
| **Feature** | **Priority** | **Release** | **Comment** |
|:------------|:-------------|:------------|:------------|
| GPX | 3 |  |  |
| Google Earth | 3 |  |  |

## Import Formats ##
| **Feature** | **Priority** | **Release** | **Comment** |
|:------------|:-------------|:------------|:------------|
| GPX | 3 |  |  |
| Google Earth | 3 |  |  |

[Google Earth/Ocean](http://www.justmagic.com/RasterChart2BSB.html) integration be a highly desirable feature set.


## Modes ##
| **Feature** | **Priority** | **Release** | **Comment** |
|:------------|:-------------|:------------|:------------|
| Up mode: North / Chart / Course / Heading | 3 |  |  |

## Navigation ##
| **Feature** | **Priority** | **Release** | **Comment** |
|:------------|:-------------|:------------|:------------|
| Add route | 1 |  |  |
| Add Mark | 1 |  |  |
| Add Range Bearing line (RBL) | 2 |  |  |
| Motion: relative or absolute | 2 |  | Ship remains centered on screen or displayed chart remains static and ship sails off the edge of the screen |
| Continuously display range and bearing from current boat position | 1 |  |  |
| Datums: WGS-84 (World Geodetic Datum 1984) only | 1 |  |  |
| Display route details in editable tabular format | 1 |  |  |

![http://marnav.googlecode.com/svn/trunk/docs/images/routeDetails.png](http://marnav.googlecode.com/svn/trunk/docs/images/routeDetails.png)

## Tide and Current ##
| View predictions for any coordinate (specify waypoint; arbitrary point; point along route) | 2 | | |
|:-------------------------------------------------------------------------------------------|:--|:|:|
| View predictions for any coordinate (specify waypoint; arbitrary point; point along route) | 2 |  |  |
| Enable/disable current vectors where known | 2 |  |  |

![http://marnav.googlecode.com/svn/trunk/docs/images/tide.png](http://marnav.googlecode.com/svn/trunk/docs/images/tide.png)
![http://marnav.googlecode.com/svn/trunk/docs/images/current.png](http://marnav.googlecode.com/svn/trunk/docs/images/current.png)

## Instruments ##
There are three types of of GPS subsystems:
  * Embedded chip set
  * External NMEA 0183 bus
  * External CAN/NMEA 2000 bus

Each of these sources of GPS data is device specifi; and existing vendors do not provide software abstraction, so all applications must write drivers to support specific devices.  The existing GPS subsystems for Linux are either buggy or very limited in scope.

We need a unified GPS driver subsystem for Moblin that will be plug and play, supporting all popular GPSes, support all three types of subsystems described above,  fault tolerant, supporting device priority and easily extensible for new GPS devices.

| **Feature** | **Priority** | **Release** | **Comment** |
|:------------|:-------------|:------------|:------------|
| Auto detect |  |  |  |
| Manual port configuration |  |  |  |
| Port troubleshooter |  |  |  |
| Data logging |  |  |  |


## Options ##
| **Feature** | **Priority** | **Release** | **Comment** |
|:------------|:-------------|:------------|:------------|
| Distances: NM & yards; Km & meters | 3 |  |  |
| Depths: Fathoms; Fathoms and feet; Feet; Meters|  |  |  |
| Heights: Feet; Meters|  |  |  |
| Speeds: Knots|  |  |  |
| Temperature: Farenheight; Celsius|  |  |  |
| Fuel: US Gallons; British Gallons; Liters|  |  |  |
| Locations: Degrees minutes and seconds|  |  |  |
| Time: 12 hour; 24 hour|  |  |  |
| Bearings: Magnetic; True|  |  |  |

## AIS ##
| **Feature** | **Priority** | **Release** | **Comment** |
|:------------|:-------------|:------------|:------------|
| Enable tracks |  |  |  |
| Specify track length (hours/minutes) |  |  |  |
| Enable predictors |  |  |  |
| Enable vessel names |  |  |  |
| Enable lost and dangerous target alerts |  |  |  |
| Dangerous target specification (distance and/or time) |  |  |  |
| Show base stations |  |  |  |
| Show aids to navigation |  |  |  |

## Misc ##
  1. A real-time drawing of a vessel outline and an arrow depicting its course vector should be refreshed once a second.
  1. The vessel orientation should line up with the course vector. The length of the course vector should scale linearly with the vessel's speed.
  1. If the vessel is not moving the arrow should disappear.
  1. Specify the vessel outline via an XPM file.
  1. More than one vessel may need to be displayed at any given moment (AIS and radar overlays will need this);
    1. An API call to define a vessel's appearance should include the vessel ID and the desired vessel outline to draw (so tugboats pulling barges look different from sailboats).
  1. Scrolling message area required