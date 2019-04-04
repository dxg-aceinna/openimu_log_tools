import math
import numpy as np

kmlstr_header = '''<?xml version = "1.0" encoding = "UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2"
     xmlns:gx = "http://www.google.com/kml/ext/2.2" > 
<Document>
      <Style id="track">
         <IconStyle>
            <color>%s</color>
            <colorMode>normal</colorMode>
            <scale> 0.50</scale>
            <Icon>
               <href>http://maps.google.com/mapfiles/kml/shapes/track.png</href>
            </Icon>
         </IconStyle>
         <LabelStyle>
            <color>%s</color>
            <scale>7.000000e-01</scale>
         </LabelStyle>
      </Style>'''
kmlstr_body = '''
   <Placemark>
      <styleUrl>#track</styleUrl>
      <Style> <IconStyle>  <heading>%f</heading> </IconStyle>  </Style>
      <Point>
         <coordinates>%.9f,%.9f,%f</coordinates>
      </Point>
   </Placemark>'''
kmlstr_end = '''
</Document>
</kml>
'''

def gen_kml(kml_file, lla, heading, color='ffff0000'):
    '''
    generate kml file.
    Args:
        kml_file: full path of the kml file
        lla: [lat lon alt] in unit [deg deg m]
        heading: in unit of deg
        color: Color and opacity (alpha) values are expressed in hexadecimal notation.
            The range of values for any one color is 0 to 255 (00 to ff). For alpha,
            00 is fully transparent and ff is fully opaque. The order of expression is
            aabbggrr, where aa=alpha (00 to ff); bb=blue (00 to ff); gg=green (00 to ff);
            rr=red (00 to ff). For example, if you want to apply a blue color with 50 percent
            opacity to an overlay, you would specify the following: <color>7fff0000</color>,
            where alpha=0x7f, blue=0xff, green=0x00, and red=0x00.
    Returns: None
    '''
    f = open(kml_file, 'w+')
    f.truncate()
    # write header
    lines = (kmlstr_header)% (color, color)
    f.write(lines)
    # write data
    ndim = lla.ndim
    if ndim == 1:
        lines = (kmlstr_body)% (heading, lla[1], lla[0], lla[2])
        f.write(lines)
    else:
        max_points = 4000.0
        step = int(math.ceil(lla.shape[0]/max_points))
        for i in range(0, lla.shape[0], step):
            lines = (kmlstr_body)% (heading[i], lla[i][1], lla[i][0], lla[i][2])
            f.write(lines)
    # write end
    f.write(kmlstr_end)
    f.close()

if __name__ == "__main__":
    kml_file = 'D:\MyDocuments\desktop\poly.kml'
    lla = np.array([[31.5,120.4,9.0], [31.6,120.5,9.0]])
    heading = np.array([0, 90])
    gen_kml(kml_file, lla, heading)
