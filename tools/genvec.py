import osmium, random, math
from geographiclib.geodesic import Geodesic
G=Geodesic.WGS84
DRIVE={"motorway","motorway_link","trunk","trunk_link","primary","primary_link",
       "secondary","secondary_link","tertiary","tertiary_link","unclassified",
       "residential","living_street","service","road"}
random.seed(11)
real=[]
for o in osmium.FileProcessor("/tmp/osm/berlin-260805.osm.pbf").with_locations().with_filter(osmium.filter.KeyFilter("highway")):
    if o.type_str()!='w' or o.tags.get("highway") not in DRIVE: continue
    pts=[]
    ok=True
    for n in o.nodes:
        if not n.location.valid(): ok=False;break
        pts.append((n.location.lat,n.location.lon))
    if not ok: continue
    for i in range(len(pts)-1):
        if random.random()<0.004:
            real.append((pts[i],pts[i+1]))
    if len(real)>25000: break
print(f"real road segments sampled: {len(real)}")

synth=[]
for lat in (0.0,30.0,52.520008,65.0,80.0):
    for d in (1.0,10.0,100.0,1000.0,10000.0,100000.0,1000000.0):
        for az in (0.0,45.0,90.0,135.0):
            r=G.Direct(lat,13.0,az,d)
            synth.append(((lat,13.0),(r['lat2'],r['lon2'])))
print(f"synthetic vectors: {len(synth)}")

with open("/home/user/openstreetmap-website-rs/tests/geodesy_vectors.tsv","w") as f:
    f.write("kind\tlat1\tlon1\tlat2\tlon2\tkarney_m\n")
    for tag,src in (("real",real),("synth",synth)):
        for (a,b) in src:
            s=G.Inverse(a[0],a[1],b[0],b[1])['s12']
            f.write(f"{tag}\t{a[0]!r}\t{a[1]!r}\t{b[0]!r}\t{b[1]!r}\t{s!r}\n")
print("vectors written")
