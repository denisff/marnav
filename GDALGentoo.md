# Emerge output #
```
 * Messages for package sci-libs/gdal-1.5.2:

 * User-specified configure options are not set.
 * If needed, set GDAL_CONFIGURE_OPTS to enable grass support.
 *
 * If you need libgrass support, then you must rebuild gdal, after
 * installing the latest Grass, and set the following option:
 *
 * GDAL_CONFIGURE_OPTS=--with-grass= emerge gdal
 *
 * GDAL is most useful with full graphics support enabled via various
 * USE flags: png, jpeg, gif, jpeg2k, etc. Also python, fits, ogdi,
 * geos, and support for either netcdf or HDF4 is available, as well as
 * grass, and mysql, sqlite, or postgres (grass support requires grass 6
 * and rebuilding gdal).  HDF5 support is now included.
 *
 * Note: tiff and geotiff are now hard depends, so no USE flags.
 * Also, this package will check for netcdf before hdf, so if you
 * prefer hdf, please emerge hdf with USE=szip prior to emerging
 * gdal.  Detailed API docs require doxygen (man pages are free).
 *
 * Check available image and data formats after building with
 * gdalinfo and ogrinfo (using the --formats switch).
 *
 * GNU info directory index is up-to-date.

```