import sys
import os 
import cv2
import tifffile
import numpy as np

try:
    from osgeo import ogr, osr, gdal, gdalconst
except:
    sys.exit('ERROR: cannot find GDAL/OGR modules')

roi = [ [],
]

def bytescaling(data, cmin=None, cmax=None, high=255, low=0):

    if data.dtype == np.uint8:
        return data
    if high > 255:
        high = 255
    if low < 0:
        low = 0
    if high < low:
        raise ValueError("`high` should be greater than or equal to `low`.")

    if cmin is None:
        cmin = data.min()
    if cmax is None:
        cmax = data.max()

    cscale = cmax - cmin

    if cscale == 0:
        cscale = 1

    scale = float(high - low) / cscale
    bytedata = (data - cmin) * scale + low
    return (bytedata.clip(low, high) + 0.5).astype(np.uint8)

def GetExtent(ds):
    """ Return list of corner coordinates from a gdal Dataset """
    xmin, xpixel, _, ymax, _, ypixel = ds.GetGeoTransform()
    width, height = ds.RasterXSize, ds.RasterYSize
    xmax = xmin + width * xpixel
    ymin = ymax + height * ypixel

    return (xmin, ymax), (xmax, ymax), (xmax, ymin), (xmin, ymin)

def ReprojectCoords(coords,src_srs,tgt_srs):
    """ Reproject a list of x,y coordinates. """
    trans_coords=[]
    transform = osr.CoordinateTransformation(src_srs, tgt_srs)
    for x,y in coords:
        x,y,z = transform.TransformPoint(x,y)
        trans_coords.append([x,y])
    return trans_coords

def geotiff_read(filename, verbose=False):
    ds = gdal.Open(filename)

    ext = GetExtent(ds)

    src_srs = osr.SpatialReference()
    src_srs.ImportFromWkt(ds.GetProjection())
    tgt_srs = src_srs.CloneGeogCS()

    geo_ext = ReprojectCoords(ext, src_srs, tgt_srs)

    return geo_ext

def find_roi(geo_ext, roi):
    for xy in roi:
        y = xy[0]
        x = xy[1]
        cor = np.array(geo_ext).T
        cor_xmax = cor[0].max()
        cor_xmin = cor[0].min()
        cor_ymax = cor[1].max()
        cor_ymin = cor[1].min()

        if (x >= cor_xmin) and (x <= cor_xmax) and (y >= cor_ymin) and (y <= cor_ymax):
            return [x, y]

    return None

def find_ratio(geo_ext, n_roi):
    aa = geo_ext[0]
    bb = geo_ext[1]
    dd = geo_ext[3]

    area1 = abs((aa[0]-n_roi[0])*(bb[1]-n_roi[1]) - (aa[1]-n_roi[1])*(bb[0]-n_roi[0]))
    AB1 = ((aa[0]-bb[0])**2 + (aa[1]-bb[1])**2)**0.5
    distance1 = area1/AB1
    to_d1 = (((aa[0]-dd[0])**2) + ((aa[1]-dd[1])**2))**0.5
    ratio1 = distance1/to_d1

    area2 = abs((aa[0]-n_roi[0])*(dd[1]-n_roi[1]) - (aa[1]-n_roi[1])*(dd[0]-n_roi[0]))
    AB2 = ((aa[0]-dd[0])**2 + (aa[1]-dd[1])**2)**0.5
    distance2 = area2/AB2
    to_d2 = (((aa[0]-bb[0])**2) + ((aa[1]-bb[1])**2))**0.5
    ratio2 = distance2/to_d2

    return [ratio1, ratio2]

def find_cor(h, w, ratio):
    y = round(ratio[0]*h)
    x = round(ratio[1]*w)
    return x, y

if __name__ == "__main__":

    main_path = ''
    out_path = ''

    path_list = [x for x in os.listdir(main_path)]
    total_num = len(path_list)

    for _, path in enumerate(path_list):

        current_path = os.path.join(main_path, path)

        if os.path.isfile(current_path):

            read_data = geotiff_read(current_path, verbose=True)
            n_roi = find_roi(read_data, roi)

            img = tifffile.imread(current_path)
            img = cv2.cvtColor(img, cv2.IMREAD_COLOR)

            img = bytescaling(img)
            b, g ,r = cv2.split(img)

            cla = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
            b = cla.apply(b)
            g = cla.apply(g)
            r = cla.apply(r)
            img = cv2.merge((b, g, r))

            if n_roi is not None:
                ratio = find_ratio(read_data, n_roi)
                x, y = find_cor(img.shape[0], img.shape[1], ratio)
                img = cv2.line(img, (x, y), (x, y), (0,0,255), 8)
            
            cv2.imwrite(os.path.join(out_path, path.split('.')[0] + '.png'), img)
        
        if (_ % 5 == 0) :            
            print(_, '/', total_num)
        

