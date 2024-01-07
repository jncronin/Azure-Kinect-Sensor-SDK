d = read.csv("nfov-calib.csv")

# make phases 0-1
d$p0 = d$p0 / pi / 2
d$p1 = d$p1 / pi / 2
d$p2 = d$p2 / pi / 2

# we want phase to increase with both increasing x and increasing distance

# expect p0 to increase ~1 for every 0.68m, and ~0.37 for every x=160
# expect p1 to increase ~1 for every 0.65m, and ~0.33 for every x=160
# expect p2 to increase ~1 for every 3.5m and ~0.1 for every x=160

# get lowest x/dist combo
min.x = min(d$x)
min.x.dist = min(d$dist[d$x == min.x])

min.p0 = d$p0[d$x == min.x & d$dist == min.x.dist]
min.p1 = d$p1[d$x == min.x & d$dist == min.x.dist]
min.p2 = d$p2[d$x == min.x & d$dist == min.x.dist]

d$expect.p0 = min.p0 + (d$x - min.x)/160 * 0.37 + (d$dist - min.x.dist) / 0.68
d$expect.p1 = min.p1 + (d$x - min.x)/160 * 0.33 + (d$dist - min.x.dist) / 0.65
d$expect.p2 = min.p2 + (d$x - min.x)/160 * 0.1 + (d$dist - min.x.dist) / 3.5

d$expect.p0.idx = -round(d$p0 - d$expect.p0)
d$expect.p1.idx = -round(d$p1 - d$expect.p1)
d$expect.p2.idx = -round(d$p2 - d$expect.p2)

d$adj.p0 = d$p0 + d$expect.p0.idx
d$adj.p1 = d$p1 + d$expect.p1.idx
d$adj.p2 = d$p2 + d$expect.p2.idx

# now perform regression
m0 = lm(adj.p0 ~ dist + x, data=d)
m1 = lm(adj.p1 ~ dist + x, data=d)
m2 = lm(adj.p2 ~ dist + x, data=d)

# get coefficients

# phase = A + Bdist + C
#  therefore dist = 1/B * ((idx + phase) + (-A) + (-C) * x)

# let c0 = 1/B, c1=-A, c2=-C
p0.coefs = coef(m0)
p1.coefs = coef(m1)
p2.coefs = coef(m2)

c0 = c(1/p0.coefs['dist'],
       1/p1.coefs['dist'],
       1/p2.coefs['dist'])

c1 = c(-p0.coefs['(Intercept)'],
       -p1.coefs['(Intercept)'],
       -p2.coefs['(Intercept)'])

c2 = c(-p0.coefs['x'],
       -p1.coefs['x'],
       -p2.coefs['x'])

nfov.coefs <- data.frame(p=0:2, c0=c0, c1=c1, c2=c2)

# get max/min idx for published range of sensor
sensor.min = 0.5
sensor.max = 5.86 # use binned max as the sensor should
  # still be able to reach this far (there is no difference
  # in hardware between the two modes as the binning occurs
  # in software)

min.df = data.frame(x=0, dist=sensor.min)
max.df = data.frame(x=639, dist=sensor.max)


nfov.coefs$min.idx = c(
  floor(predict(m0, min.df)),
  floor(predict(m1, min.df)),
  floor(predict(m2, min.df))
)

nfov.coefs$max.idx = c(
  ceiling(predict(m0, max.df)),
  ceiling(predict(m1, max.df)),
  ceiling(predict(m2, max.df))
)
