"""
agent template for the nav challenge
"""
import math, random, time #time for slowing down sim for bugfix
class Agent:
  def __init__(self, cfg:dict):

    """cfg keys: width_m, height_m, resolution, robot_radius, v_max, a_max, dt, sense_cells, goal_tol, goal (x, y)."""
    self.cfg = cfg
    self.goal = cfg["goal"]
    self.theta = 10 #Because it will be confined from -PI to PI later, 10 is a value it will never be so I can "check" if its been initialized.

  def sightLineAdjustsTheta(self, vision, sightAngle, exp, sight):
    gx, gy = self.goal
    x, y = self.pose
    deltaTheta = 0
    clear = True
    coords = set() #the coordinates infront of blob (adds (0, 0) to start)
    for k in range(vision):
      coords.add((round(k*math.cos(self.theta+sightAngle))+25, round(k*math.sin(self.theta+sightAngle))+25))
    for xc, yc in coords:
      if sight[50-yc][xc] == '#':
        distMult = 1-(math.dist((25, 25), (xc, yc))/vision) #the further the distance the less the change
        distMult = distMult*exp
        deltaTheta -= math.pi*0.2*(distMult) * math.sqrt(sightAngle**2)/sightAngle #sign(sightAngle)
        clear = False
        break
      #this code runs if the aqua/pink sightline didnt see anything
    if round(abs(math.degrees(sightAngle))) == 35 and clear:
      ga = math.atan2(gy-y, gx-x) #goal angle
      pi = math.pi
      #bounding fuction((a)+pi)%(2*pi)-pi)
      angleAway = abs((ga-self.theta+pi)%(2*pi)-pi)
      rayAngleAway = abs((ga-self.theta-sightAngle+pi)%(2*pi)-pi)
      if rayAngleAway < angleAway:
        deltaTheta += sightAngle * 0.45


    return(coords, deltaTheta)



  def step(self, pose:tuple[float, float], scan:tuple[int, int, list[str]]) -> tuple[float,
 float]:
    #fps = 120 #doesnt work exactly like fps but close enough lol
   # time.sleep(1/fps)

    ##speed
    vx = 0
    vy = 0
    ##sight
    cx0, cy0, rows = scan
    midScan = len(rows) // 2
    midRows = rows[midScan-25 : midScan+ 26]
    sight = []
    for row in reversed(midRows): #runs through rows backwards so view isnt upside down
        midCols = row[midScan-25 : midScan+26]
        sight.append(list(midCols))


    ##position velocity and angle
    x, y = pose
    self.pose = pose
    gx, gy = self.goal
    dx = gx - x
    dy = gy - y
    d = (dx**2+dy**2)**0.5
    if self.theta == 10 or d < 2: #(if its never been modified or close enoguh to goal)
      self.theta = math.atan2(dy, dx)

  ##sight lines
    RedVision = 20 #Vision that the robot will act upon seeing
    BlueVision = 20
    GreenVision = 13
    YellowVision = 13
    BlackVision = 15
    PinkVision = 15
    AquaVision = 15
    #       XDT stands for [color] delta theta      vision distance, angle from forward, pass sight,
    RedCoords, RDT = self.sightLineAdjustsTheta(RedVision, math.radians(15), 1.1, sight)
    BlueCoords, BDT = self.sightLineAdjustsTheta(BlueVision, math.radians(-15), 1.1, sight)

    PinkCoords, PDT = self.sightLineAdjustsTheta(PinkVision, math.radians(35), 1.3, sight)
    AquaCoords, ADT = self.sightLineAdjustsTheta(AquaVision, math.radians(-35), 1.3, sight)

    GreenCoords, GDT = self.sightLineAdjustsTheta(GreenVision, math.radians(-75), 0.77, sight)
    YellowCoords, YDT = self.sightLineAdjustsTheta(YellowVision, math.radians(75), 0.77, sight)
    BlackCoords, BLDT = self.sightLineAdjustsTheta(BlackVision, math.radians(-1), 1.5, sight)


    self.theta += RDT + BDT + GDT*2 + YDT*2 + BLDT + PDT + ADT
    speedMultiplier = 1
    if abs(RDT + BDT + GDT*2 + YDT*2 + BLDT + PDT + ADT) > 0.02: #if turning fast slow down
      speedMultiplier = 0.75

    #slight pull towards that juicy fruit
    fruitSmellMultiplier = 0.08 #(How tasty that fruit be smelling measured in smells per second)
    if(d < 7):
      self.theta += (math.atan2(dy, dx) - self.theta)*fruitSmellMultiplier
    else:
     angleChange = (math.atan2(dy, dx) - self.theta)
     boundAngle = ((angleChange)+math.pi)%(2*math.pi)-math.pi #(-pi, pi)
     self.theta += boundAngle*fruitSmellMultiplier

  ##printing sights
   # print(scan)
    for i in range(51): #51 = 1 + 2*sense_cells (21)
      for j in range (51):
        c = sight[i][j] #char from scan
        o = 'o' #prints an o unless was not in ALL the colors, then prints a white scan char
        if(j, 50-i) in RedCoords:
          print('\033[31m', end='') #turn text red
        elif(j, 50-i) in BlueCoords:
          print('\033[34m', end='')
        elif(j, 50-i) in GreenCoords:
          print('\033[32m', end='')
        elif(j, 50-i) in YellowCoords:
          print('\033[33m', end='')
        elif(j, 50-i) in BlackCoords:
          print('\033[30m', end='')
        elif(j, 50-i) in PinkCoords:
          print('\033[95m', end='')
        elif(j, 50-i) in AquaCoords:
          print('\033[96m', end='')




        else:
          print('\033[0m', end='')  #turn text white
          o = c #only if all else fails print the background char

        if j != 50:
          print(o + ' ', end='')
        else:
          print(o)

  ##return
    #edit velocities in response to self.theta
    vx = 2*math.cos(self.theta) * speedMultiplier
    vy = 2*math.sin(self.theta) * speedMultiplier
    return (vx, vy)


    """
    called once per tick.
    pose: (x, y) metres from SLAM, ~2 cm gaussian noise.
    scan: (cx0, cy0, rows) -- a (2*sense_cells+1)^2 window of '#'/'.' around the robot. rows[j][i] is cell (cx0+i, cy0+j).
          everything in the window is observed, nothing outside it is.
          the window origin comes from the noisy pose, so walls can land one cell off between scans.
    returns: (vx, vy) world-frame velocity command in m/s. sim clamps speed and acceleration.
    """


  def debug(self) -> dict:
    """
    optional, for `harness.py --viz` only.
    keys: blocked (cells), free (cells), path ([(x, y), ...]).
    """
    return {}
