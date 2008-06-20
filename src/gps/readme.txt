Hi Michael,

The problems I had with gpsd is that the logic was not perfect. There where race
conditions that caused data to get lost and also for it to hang. It also seemed
somewhat silly that it runs itself as a client and server which seemed to add
complexity that just isn't necessary for something that processes data at a
maybe up to 20Hz rate.

So what I did was to extract the NMEA parsing stuff from the GPSD project and
fix the ring buffer problems. I couldn't convince to the owners that these
problems where legitimate so I do not think they ever accepted my changes.

I ended up with several customized objects that worked with specific GPSs that
we owned. I am attaching one that we used on the boat. This object is not very
pretty but it is fairly straight forward. It knows how to parse the sentences
from a Sirf3 type GPS. The constructor needs the device name to open for
communication. Once the object is created and it has synced with the GPS then
you simply call the run() function when you have some spare cycles. This will
check for new data and parse it. Keeping it very simple, but not so safe, you
simply extract what you want from the object variables, you will find the
latitude, longitude and various other variables in the header file.

We had a slightly more sophisticated version for the car project that would
broadcast the data via UDP but I don't think that is pratical for most
applications.

I am attaching the code, you can do what you want with it. It is ugly......

Curt
cmeyers@willowgarage.com
