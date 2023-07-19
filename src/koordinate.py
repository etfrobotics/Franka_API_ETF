import re
import math




class Coords():

    def __init__(self,s,flag_calc=0,flag_direction=1,per=1, board_origin = [0.434919172465, 0.00149003666781], board_orientation = 0):
        self.flag_calc = flag_calc
        self.percentage = per
        self.flag_direction = flag_direction
        self.joints, self.position, self.orientation, self.rpy, self.pos_board_cs = self.extract_data(s)
        self.transform_coordinates(board_origin, board_orientation)

    @classmethod
    def from_abs_coords(cls,pos,rpy):
      return cls()

    def extract_data(self,s):
        # Extract joints
        joints = re.findall(r'-?\d+\.\d+', s.split('\n')[0])
        
        # Extract position
        position = []
        position_regex = r'position:\s*\n\s*x:\s*(-?\d+\.\d+)\s*\n\s*y:\s*(-?\d+\.\d+)\s*\n\s*z:\s*(-?\d+\.\d+)'
        position_match = re.search(position_regex, s)
        if position_match:
            position = [float(position_match.group(1)), float(position_match.group(2)), float(position_match.group(3))]
        
        # Extract orientation
        orientation = []
        orientation_regex = r'orientation:\s*\n\s*x:\s*(-?\d+\.\d+)\s*\n\s*y:\s*(-?\d+\.\d+)\s*\n\s*z:\s*(-?\d+\.\d+)\s*\n\s*w:\s*(-?\d+\.\d+)'
        orientation_match = re.search(orientation_regex, s)
        if orientation_match:
            orientation = [float(orientation_match.group(1)), float(orientation_match.group(2)), float(orientation_match.group(3)), float(orientation_match.group(4))]
        
        # Extract rpy
        rpy = []
        rpy_regex = r'RPY = \[(.*?)\]'
        rpy_match = re.search(rpy_regex, s)
        if rpy_match:
            rpy_str = rpy_match.group(1).strip()
            rpy = [float(x.strip()) for x in rpy_str.split(',')]

        joints = [float(joint) for joint in joints]
        pos_board_cs = [position[0] - 0.44798833408, position[1] - 0.0983415023025, position[2] - 0.21330782485]

        if self.flag_calc:
          if self.flag_direction:
            pos_board_cs[0] += 0.029 * (1-self.percentage)
          else:
            pos_board_cs[0] -= 0.029 * self.percentage
        
        return joints, position, orientation, rpy, pos_board_cs
    
    # define a function to transform coordinates
    def transform_coordinates(self, board_origin, board_orientation):
  
        # calculate the new coordinates in the robot's coordinate system
        robot_x = math.cos(board_orientation) * self.pos_board_cs[0] + math.sin(board_orientation) * self.pos_board_cs[1] + board_origin[0]
        robot_y = -math.sin(board_orientation) * self.pos_board_cs[0] + math.cos(board_orientation) * self.pos_board_cs[1] + board_origin[1]

        self.position[0] = robot_x
        self.position[1] = robot_y
        self.rpy[2] -= board_orientation


 

def calculate_coordinates(board_origin, board_orientation):

  print("Racunam")
  # start_button_approach = Coords('''joints = [0.271870232572 0.159104162302 -0.0736630360735 -2.39942199593 0.0135551755486 2.594682936 0.966274300607]
  # panda_link8 pose = [
  # header: 
  #   seq: 0
  #   stamp: 
  #     secs: 1681995541
  #     nsecs: 817485094
  #   frame_id: "/world"
  # pose: 
  #   position: 
  #     x: 0.443943242496
  #     y: 0.0984648762057
  #     z: 0.253280172136
  #   orientation: 
  #     x: -0.925061326361
  #     y: 0.379369154086
  #     z: -0.0165099124738
  #     w: 0.00824682912067 ]
  # panda_link8 RPY = [-3.1138000993448074, -0.024285958224517384, -0.7787179084494085]''', 0,1,1,board_origin, board_orientation)

  print("Izracunao")

  start_button_press = Coords('''joints = [0.412597410983 0.164181102811 -0.183924663199 -2.55672592441 0.0763932341404 2.7707845938 1.0086144719]
  panda_link8 pose = [
  header: 
  seq: 0
  stamp: 
    secs: 1682789626
    nsecs: 742530107
  frame_id: "/world"
  pose: 
  position: 
    x: 0.44798833408
    y: 0.0983415023025
    z: 0.21330782485
  orientation: 
    x: -0.911288604869
    y: 0.410895008805
    z: -0.0219114544515
    w: 0.0154356256343 ]
  panda_link8 RPY = [-3.0954165787477335, -0.027253823298962938, -0.7853981634]''', 0,1,1,board_origin, board_orientation)

  # stare izmerene pozicije, JOINTOVI RADE SAMO ZA OVE KOORDINATE
  #  x: 0.453394660416
  #  y: 0.103808170816

  # start_button_depart = Coords('''joints = [0.276717146995 0.203739311009 -0.076398581921 -2.34849551667 0.0352137333109 2.55607748323 0.989105987273]
  # panda_link8 pose = [
  # header: 
  #   seq: 0
  #   stamp: 
  #     secs: 1681997278
  #     nsecs: 417503118
  #   frame_id: "/world"
  # pose: 
  #   position: 
  #     x: 0.455106577216
  #     y: 0.101508780203
  #     z: 0.246053729207
  #   orientation: 
  #     x: -0.917816821177
  #     y: 0.396992130571
  #     z: -0.000773307508641
  #     w: 0.0029888167333 ]
  # panda_link8 RPY = [-3.1354898277608774, 0.0009522010419122964, -0.8164546350056683]''', 0,1,1,board_origin, board_orientation)

  slider_snapshot = Coords('''joints = [0.244976526262 0.261018288487 -0.106685642136 -2.30027047361 -0.0613137070914 2.55284945779 -0.590892073918]
  panda_link8 pose = [
  header: 
    seq: 0
    stamp: 
      secs: 1682868395
      nsecs:  60878992
    frame_id: "/world"
  pose: 
    position: 
      x: 0.522958831229
      y: 0.0728090089635
      z: 0.234111707548
    orientation: 
      x: 0.923740660524
      y: 0.381769526341
      z: -0.0105885407288
      w: 0.0290362472913 ]
  panda_link8 RPY = [3.0959788693983064, 0.04174856071909401, 0.7828622823725659]''', 0,1,1,board_origin, board_orientation)

  # slider_approach = Coords('''joints = [0.192749941181 0.478827290359 -0.0914787598043 -1.92670335761 0.0393318030967 2.40903549168 -0.694282036432]
  # panda_link8 pose = [
  # header: 
  #   seq: 0
  #   stamp: 
  #     secs: 1681997806
  #     nsecs: 137871026
  #   frame_id: "/world"
  # pose: 
  #   position: 
  #     x: 0.572817422735
  #     y: 0.0660710884705
  #     z: 0.240704214899
  #   orientation: 
  #     x: 0.925495860062
  #     y: 0.378664963636
  #     z: 0.00053276232977
  #     w: 0.00836507540487 ]
  # panda_link8 RPY = [3.125714021364724, 0.0053593690741117715, 0.7766893228613874]''', 0,1,1,board_origin, board_orientation)

  slider_push = Coords('''joints = [0.334066071347 0.383714202111 -0.21081906773 -2.1869625966 0.056998641396 2.60584245019 -0.652971490034]
  panda_link8 pose = [
  header: 
  seq: 0
  stamp: 
    secs: 1682790182
    nsecs: 909615993
  frame_id: "/world"
  pose: 
  position: 
    x: 0.560357284134
    y: 0.0675537744405
    z: 0.217556904856
  orientation: 
    x: 0.931013562477
    y: 0.363499840159
    z: 0.0154826000016
    w: 0.0290155438607 ]
  panda_link8 RPY = [3.0762604511881086, -0.007740819463268916, 0.7447027157679146]''', 0,1,1,board_origin, board_orientation)

  # slider_push_end = Coords('''joints = [0.208819320562 0.524114647401 -0.0981062594283 -1.92132717429 0.0400320090088 2.40900986009 -0.687297970638]
  # panda_link8 pose = [
  # header: 
  #   seq: 0
  #   stamp: 
  #     secs: 1681997985
  #     nsecs: 657588958
  #   frame_id: "/world"
  # pose: 
  #   position: 
  #     x: 0.540325191028
  #     y: 0.0714759196786
  #     z: 0.214844583332
  #   orientation: 
  #     x: 0.924709309765
  #     y: 0.380132709025
  #     z: -0.0193019382588
  #     w: 0.00626507325405 ]
  # panda_link8 RPY = [-3.138509937810762, 0.040464522904350535, 0.7801138176377815]''', 0,1,1,board_origin, board_orientation)

  # slider_switch_side = Coords('''joints = [0.36017889603 0.218619363605 -0.249109327383 -2.26191416175 0.0783722798738 2.43642430915 -0.730342694728]
  # panda_link8 pose = [
  # header: 
  #   seq: 0
  #   stamp: 
  #     secs: 1682598353
  #     nsecs: 424556016
  #   frame_id: "/world"
  # pose: 
  #   position: 
  #     x: 0.530393287681
  #     y: 0.0586729317221
  #     z: 0.263940275547
  #   orientation: 
  #     x: -0.923713432377
  #     y: -0.382658886574
  #     z: 0.0176819125038
  #     w: 0.00360850969357 ]
  # panda_link8 RPY = [-3.12138370188965, 0.029908996565700563, 0.785793973225024]''', 0,1,1,board_origin, board_orientation)

  # slider_push_back_prepare = Coords('''joints = [0.367889961364 0.261126254893 -0.235156569871 -2.36863121072 0.0930403406277 2.58111010207 -0.731736562856]
  # panda_link8 pose = [
  # header: 
  #   seq: 0
  #   stamp: 
  #     secs: 1682598426
  #     nsecs:  57832956
  #   frame_id: "/world"
  # pose: 
  #   position: 
  #     x: 0.500994222218
  #     y: 0.0619897419343
  #     z: 0.245200528454
  #   orientation: 
  #     x: 0.922588599887
  #     y: 0.385235367645
  #     z: -0.0205901902273
  #     w: 0.00017589805162 ]
  # panda_link8 RPY = [-3.126039186877564, 0.038140453331730766, 0.791388137230557]''', 0,1,1,board_origin, board_orientation)

  slider_push_back = Coords('''joints = [0.365462654269 0.250319810029 -0.219253089769 -2.42041105082 0.0332876862321 2.70343168099 -0.655756284205]
  panda_link8 pose = [
  header: 
    seq: 0
    stamp: 
      secs: 1682790690
      nsecs: 475886106
    frame_id: "/world"
  pose: 
    position: 
      x: 0.493420125416
      y: 0.0682959461525
      z: 0.212955051957
    orientation: 
      x: 0.924597312242
      y: 0.379935502414
      z: 0.0141269232975
      w: 0.0238590494018 ]
  panda_link8 RPY = [3.086705425315451, -0.00800149170423337, 0.7799933119358475]''', 0,1,1,board_origin, board_orientation)

  # slider_push_back_end = Coords('''joints = [0.367889961364 0.261126254893 -0.235156569871 -2.36863121072 0.0930403406277 2.58111010207 -0.731736562856]
  # panda_link8 pose = [
  # header: 
  #   seq: 0
  #   stamp: 
  #     secs: 1682598426
  #     nsecs:  57832956
  #   frame_id: "/world"
  # pose: 
  #   position: 
  #     x: 0.529994222218
  #     y: 0.0619897419343
  #     z: 0.215200528454
  #   orientation: 
  #     x: 0.922588599887
  #     y: 0.385235367645
  #     z: -0.0205901902273
  #     w: 0.00017589805162 ]
  # panda_link8 RPY = [-3.126039186877564, 0.038140453331730766, 0.791388137230557]''', 0,1,1,board_origin, board_orientation)

  # test_port_take_out_approach = Coords('''joints = [0.265749519336 0.256804145181 -0.16912505799 -2.37014765241 0.00282145871888 2.60552522951 0.873920564855]
  # panda_link8 pose = [
  # header: 
  #   seq: 0
  #   stamp: 
  #     secs: 1682006253
  #     nsecs: 619752883
  #   frame_id: "/world"
  # pose: 
  #   position: 
  #     x: 0.454854020562
  #     y: 0.0466532195967
  #     z: 0.317059977973
  #   orientation: 
  #     x: 0.925679281007
  #     y: -0.377638608123
  #     z: 0.00180497800513
  #     w: 0.0224430930026 ]
  # panda_link8 RPY = [3.1013888799747575, -0.020294433195789233, -0.7742742344255571]''', 0,1,1,board_origin, board_orientation)

  test_port_take_out = Coords('''joints = [0.338127919605 0.130485322574 -0.234318823982 -2.56966125759 -0.0193603740044 2.72324884706 0.906230387083]
  panda_link8 pose = [
  header: 
    seq: 0
    stamp: 
      secs: 1682790961
      nsecs: 242918968
    frame_id: "/world"
  pose: 
    position: 
      x: 0.450169677267
      y: 0.0450743558555
      z: 0.220324328506
    orientation: 
      x: 0.924184562993
      y: -0.381237778094
      z: 0.0208130626314
      w: 0.0103666052437 ]
  panda_link8 RPY = [3.1382905635074017, -0.046385694980998184, -0.7824272147614435]''', 0,1,1,board_origin, board_orientation)

  # test_port_take_out_depart = Coords('''joints = [0.265749519336 0.256804145181 -0.16912505799 -2.37014765241 0.00282145871888 2.60552522951 0.873920564855]
  # panda_link8 pose = [
  # header: 
  #   seq: 0
  #   stamp: 
  #     secs: 1682006253
  #     nsecs: 619752883
  #   frame_id: "/world"
  # pose: 
  #   position: 
  #     x: 0.454854020562
  #     y: 0.0466532195967
  #     z: 0.317059977973
  #   orientation: 
  #     x: 0.925679281007
  #     y: -0.377638608123
  #     z: 0.00180497800513
  #     w: 0.0224430930026 ]
  # panda_link8 RPY = [3.1013888799747575, -0.020294433195789233, -0.7742742344255571]''', 0,1,1,board_origin, board_orientation)

  # test_port_plug_in_prepare = Coords('''joints = [0.909175806677 0.218485665211 -0.784565117386 -2.4826473218 0.391177351199 2.65509583779 0.602036247797]
  # panda_link8 pose = [
  # header: 
  #   seq: 0
  #   stamp: 
  #     secs: 1682353967
  #     nsecs: 674159049
  #   frame_id: "/world"
  # pose: 
  #   position: 
  #     x: 0.475049754698
  #     y: 0.0387101359142
  #     z: 0.23202802747
  #   orientation: 
  #     x: -0.919358581859
  #     y: 0.392491997072
  #     z: -0.0137940008917
  #     w: 0.0232283390882 ]
  # panda_link8 RPY = [-3.0880222160144855, -0.007123065203446392, -0.8071777542758173]''', 0,1,1,board_origin, board_orientation)

  test_port_plug_in = Coords('''joints = [0.333021631203 0.151382270533 -0.234261829935 -2.48506457533 -0.0196069813992 2.66923524194 0.906817508729]
  panda_link8 pose = [
  header: 
    seq: 0
    stamp: 
      secs: 1682791120
      nsecs: 842514038
    frame_id: "/world"
  pose: 
    position: 
      x: 0.472426941746
      y: 0.0466269562202
      z: 0.234755614071
    orientation: 
      x: 0.923091214319
      y: -0.383509347143
      z: 0.0266824238622
      w: 0.0105469880809 ]
  panda_link8 RPY = [-3.1405893997814407, -0.0573866303012271, -0.7875714377198617]''', 0,1,1,board_origin, board_orientation)

  # test_port_plug_in_approach = Coords('''joints = [0.264208814265 0.304316648843 -0.169010053273 -2.29421158995 0.000142696448799 2.59029946937 0.91022988131]
  # panda_link8 pose = [
  # header: 
  #   seq: 0
  #   stamp: 
  #     secs: 1682006388
  #     nsecs: 453110933
  #   frame_id: "/world"
  # pose: 
  #   position: 
  #     x: 0.477848611456
  #     y: 0.048641052733
  #     z: 0.31822850704
  #   orientation: 
  #     x: 0.919276196545
  #     y: -0.392801947488
  #     z: 0.01040096001
  #     w: 0.0230157456126 ]
  # panda_link8 RPY = [3.107415458977034, -0.037206408850144125, -0.8069751767341896]''', 0,1,1,board_origin, board_orientation)

  # hatch_approach = Coords('''joints = [0.514404692487 0.304153530904 -0.590411468665 -2.32450332528 0.373428094648 2.5221574481 1.00313951457]
  # panda_link8 pose = [
  # header: 
  #   seq: 0
  #   stamp: 
  #     secs: 1682172737
  #     nsecs: 550496101
  #   frame_id: "/world"
  # pose: 
  #   position: 
  #     x: 0.463229098735
  #     y: -0.0494928284052
  #     z: 0.226860635986
  #   orientation: 
  #     x: -0.773846767021
  #     y: 0.632846511438
  #     z: 0.0230585742129
  #     w: 0.0116093190049 ]
  # panda_link8 RPY = [3.1303644692960133, 0.05040256159503262, -1.3712798459268292]''', 0,1,1,board_origin, board_orientation)

  hatch_grasped = Coords('''joints = [0.181547513567 0.159829044983 -0.263700385027 -2.53548162665 -0.0194449416165 2.72285569482 0.655434851901]
  panda_link8 pose = [
  header: 
    seq: 0
    stamp: 
      secs: 1682791453
      nsecs: 576095104
    frame_id: "/world"
  pose: 
    position: 
      x: 0.461671274916
      y: -0.0405449318274
      z: 0.219105229497
    orientation: 
      x: 0.936176225042
      y: -0.350273275442
      z: 0.0226879074136
      w: 0.0191824668636 ]
  panda_link8 RPY = [3.121542974810188, -0.0559550763581593, -0.7853981634]''', 0,1,1,board_origin, board_orientation)

  # hatch_grasped_turn = Coords('''joints = [-0.0984209628432 0.153450097814 0.00160886362701 -2.53567996195 -0.00301316570624 2.72171841132 0.697273489986]
  # panda_link8 pose = [
  # header: 
  #   seq: 0
  #   stamp: 
  #     secs: 1682358538
  #     nsecs:  78157901
  #   frame_id: "/world"
  # pose: 
  #   position: 
  #     x: 0.461192457278
  #     y: -0.0446517657652
  #     z: 0.219314316164
  #   orientation: 
  #     x: -0.922610173631
  #     y: 0.385388855372
  #     z: -0.0157211658538
  #     w: 0.00432927379879 ]
  # panda_link8 RPY = [-3.121486801615278, -0.025667113930028754, -0.7916218546685341]''', 0,1,1,board_origin, board_orientation)

  hatch_lift_15 = Coords('''joints = [0.349702960579 0.257248760603 -0.427217607424 -2.25466871399 0.0416790076175 2.19330387408 0.680126900863]
  panda_link8 pose = [
  header: 
    seq: 0
    stamp: 
      secs: 1682792362
      nsecs: 404783964
    frame_id: "/world"
  pose: 
    position: 
      x: 0.503683938623
      y: -0.0413709372395
      z: 0.238259672285
    orientation: 
      x: 0.912320553102
      y: -0.380496932242
      z: -0.128483508097
      w: 0.0799079538498 ]
  panda_link8 RPY = [2.8916684107764685, 0.17450945787952798, -0.812217060405874]''', 0,1,1,board_origin, board_orientation)

  hatch_lift_30 = Coords('''joints = [0.340846927237 0.370637051444 -0.425310891407 -2.05478964558 0.0580432096053 1.90772267366 0.66429624687]
  panda_link8 pose = [
  header: 
    seq: 0
    stamp: 
      secs: 1682792656
      nsecs:  86627006
    frame_id: "/world"
  pose: 
    position: 
      x: 0.530837446777
      y: -0.0406814281175
      z: 0.242261813051
    orientation: 
      x: 0.894516545968
      y: -0.374487222665
      z: -0.213915257326
      w: 0.117642389187 ]
  panda_link8 RPY = [2.7432376558124667, 0.29903021026285936, -0.8537627571914687]''', 0,1,1,board_origin, board_orientation)

  hatch_lift_60 = Coords('''joints = [0.307143396896 0.589246270816 -0.418233976013 -1.73906774817 0.0806413663189 1.51549680305 0.635969937833]
  panda_link8 pose = [
  header: 
    seq: 0
    stamp: 
      secs: 1682792785
      nsecs: 607795000
    frame_id: "/world"
  pose: 
    position: 
      x: 0.566246155485
      y: -0.0406018180548
      z: 0.232435387667
    orientation: 
      x: 0.85203842577
      y: -0.36187021213
      z: -0.336665010427
      w: 0.172444603679 ]
  panda_link8 RPY = [2.4961865596002473, 0.4655231418763083, -0.9614594552474505]''', 0,1,1,board_origin, board_orientation)

  hatch_lift_75 = Coords('''joints = [0.218994779298 0.938257503687 -0.401949097759 -1.22713762354 0.100020690977 0.994385537191 0.592000429821]
  panda_link8 pose = [
  header: 
    seq: 0
    stamp: 
      secs: 1682792926
      nsecs: 555113077
    frame_id: "/world"
  pose: 
    position: 
      x: 0.608550824537
      y: -0.0427549172907
      z: 0.210696305494
    orientation: 
      x: 0.774575274177
      y: -0.33700907745
      z: -0.483624704664
      w: 0.22927095627 ]
  panda_link8 RPY = [2.1308386987114325, 0.6368666163159243, -1.1816033738083915]''', 0,1,1,board_origin, board_orientation)

  hatch_lift_90 = Coords('''joints = [0.122077432535 1.20403351046 -0.37052108558 -0.780784576642 0.108292599892 0.692329905573 0.592734652675]
  panda_link8 pose = [
  header: 
    seq: 0
    stamp: 
      secs: 1682793054
      nsecs: 874506950
    frame_id: "/world"
  pose: 
    position: 
      x: 0.633870899052
      y: -0.0440645120123
      z: 0.19564987795
    orientation: 
      x: 0.738019123069
      y: -0.324837840444
      z: -0.536473034035
      w: 0.249007700993 ]
  panda_link8 RPY = [1.9679861254252995, 0.6816509559561161, -1.2924638527632963]''', 0,1,1,board_origin, board_orientation)

  hatch_lift_100 = Coords('''joints = [0.108535414256 1.21394172608 -0.353084395109 -0.725216524238 0.11358990496 0.745491664478 0.612478034739]
  panda_link8 pose = [
  header: 
    seq: 0
    stamp: 
      secs: 1682793219
      nsecs: 948780059
    frame_id: "/world"
  pose: 
    position: 
      x: 0.648437432567
      y: -0.0416108745784
      z: 0.200429069975
    orientation: 
      x: 0.764352959454
      y: -0.33137113769
      z: -0.49911185927
      w: 0.238422051027 ]
  panda_link8 RPY = [2.079902980134139, 0.649744367237863, -1.2085587356854217]''', 0,1,1,board_origin, board_orientation)

  hatch_lift_depart = Coords('''joints = [0.107168733154 1.19216665227 -0.352647287097 -0.716488370009 0.1853263882 1.16116489862 0.651573149554]
  panda_link8 pose = [
  header: 
    seq: 0
    stamp: 
      secs: 1682793317
      nsecs: 798928022
    frame_id: "/world"
  pose: 
    position: 
      x: 0.70858405396
      y: -0.03824241973
      z: 0.202339702731
    orientation: 
      x: 0.862512295282
      y: -0.361280113457
      z: -0.313878846935
      w: 0.164405868373 ]
  panda_link8 RPY = [2.5433647757084965, 0.4363772023894484, -0.9298436884241175]''', 0,1,1,board_origin, board_orientation)

  # cable_approach = Coords('''joints = [0.202170961477 0.0804114401685 -0.417244245114 -2.53881864177 0.0598394061723 2.71265395346 0.552742318503]
  # panda_link8 pose = [
  # header: 
  #   seq: 0
  #   stamp: 
  #     secs: 1682361777
  #     nsecs: 603135108
  #   frame_id: "/world"
  # pose: 
  #   position: 
  #     x: 0.457122721786
  #     y: -0.104073465732
  #     z: 0.254700661029
  #   orientation: 
  #     x: -0.916315381728
  #     y: 0.397248464643
  #     z: -0.0502257987947
  #     w: 0.00609488990004 ]
  # panda_link8 RPY = [-3.0903052166568656, -0.08730818140015416, -0.8203943158329213]''', 0,1,1,board_origin, board_orientation)

  cable_grab = Coords('''joints = [0.123968671809 0.20600802755 -0.311686978045 -2.46212787739 -0.0175176675807 2.62793402963 0.600727843634]
  panda_link8 pose = [
  header: 
    seq: 0
    stamp: 
      secs: 1682793628
      nsecs:  59730052
    frame_id: "/world"
  pose: 
    position: 
      x: 0.469573183141
      y: -0.0932599124007
      z: 0.215675683591
    orientation: 
      x: 0.926509617253
      y: -0.374266160163
      z: -0.00776256462593
      w: 0.0380067505145 ]
  panda_link8 RPY = [3.0652692282630625, -0.01406146968808864, -0.767290947211288]''', 0,1,1,board_origin, board_orientation)

  cable_take_out = Coords('''joints = [0.123968671809 0.20600802755 -0.311686978045 -2.46212787739 -0.0175176675807 2.62793402963 0.600727843634]
  panda_link8 pose = [
  header: 
    seq: 0
    stamp: 
      secs: 1682793628
      nsecs:  59730052
    frame_id: "/world"
  pose: 
    position: 
      x: 0.449573183141
      y: -0.0932599124007
      z: 0.215675683591
    orientation: 
      x: 0.926509617253
      y: -0.374266160163
      z: -0.00776256462593
      w: 0.0380067505145 ]
  panda_link8 RPY = [3.0652692282630625, -0.01406146968808864, -0.767290947211288]''', 0,1,1,board_origin, board_orientation)

  # cable_depart = Coords('''joints = [0.367523164797 -0.222238154093 -0.589983605878 -2.43371566088 -0.142400211442 2.2743727845 0.580763004811]
  # panda_link8 pose = [
  # header: 
  #   seq: 0
  #   stamp: 
  #     secs: 1682362392
  #     nsecs: 943789005
  #   frame_id: "/world"
  # pose: 
  #   position: 
  #     x: 0.452455328449
  #     y: -0.10610279288
  #     z: 0.377209201093
  #   orientation: 
  #     x: -0.938738972639
  #     y: 0.344202004796
  #     z: -0.0143905315023
  #     w: 0.00932918783659 ]
  # panda_link8 RPY = [-3.1141569175243, -0.020591434238175686, -0.7031788123242041]''', 0,1,1,board_origin, board_orientation)

  cable_plug_in_approach = Coords('''joints = [-0.506314704784 0.0823964123496 0.0489897385019 -2.44676241309 1.68905913737 1.73867231187 1.4637706234]
  panda_link8 pose = [
  header: 
    seq: 0
    stamp: 
      secs: 1682794409
      nsecs: 246963977
    frame_id: "/world"
  pose: 
    position: 
      x: 0.457432822631
      y: -0.124547705664
      z: 0.295299512493
    orientation: 
      x: -0.0238077072511
      y: 0.729641057894
      z: 0.530128038522
      w: 0.431301962068 ]
  panda_link8 RPY = [1.658060172354127, 0.7136902636222886, 2.5484573970883306]''', 0,1,1,board_origin, board_orientation)

  cable_plug_in_prepare = Coords('''joints = [1.84341604375 -0.647743374503 -2.32900056464 -2.41061851595 1.58900393977 2.05443844134 0.994462769222]
  panda_link8 pose = [
  header: 
    seq: 0
    stamp: 
      secs: 1682878982
      nsecs: 406773090
    frame_id: "/world"
  pose: 
    position: 
      x: 0.43553669126
      y: -0.112401972026
      z: 0.147479723695
    orientation: 
      x: -0.0226335251869
      y: 0.71550104717
      z: 0.508235937055
      w: 0.484716634436 ]
  panda_link8 RPY = [1.5900221636953722, 0.7934286382397632, 2.4253824673228315]''', 0,1,1,board_origin, board_orientation)

  cable_plugged_in = Coords('''joints = [0.253399661344 0.502619115616 -0.659110402584 -2.51377550528 2.12200857108 1.47743412572 1.16592534402]
  panda_link8 pose = [
  header: 
    seq: 0
    stamp: 
      secs: 1682439261
      nsecs: 145445108
    frame_id: "/world"
  pose: 
    position: 
      x: 0.430186283994
      y: -0.105507962895
      z: 0.148444789853
    orientation: 
      x: 0.0408692269516
      y: 0.725334807565
      z: 0.479514785003
      w: 0.492224231612 ]
  panda_link8 RPY = [1.6461671823306239, 0.7407698969397285, 2.3377474686073714]''', 0,1,1,board_origin, board_orientation)

  cable_wrap_prepare = Coords('''joints = [0.557176181312 -0.234949388489 -0.522997144604 -0.965315436063 -0.170102409449 0.890951387617 0.651829693669]
  panda_link8 pose = [
  header: 
    seq: 0
    stamp: 
      secs: 1682439744
      nsecs: 556123971
    frame_id: "/world"
  pose: 
    position: 
      x: 0.275118654316
      y: -0.0743967191197
      z: 0.894290259079
    orientation: 
      x: -0.932624827528
      y: 0.355573283971
      z: -0.0591329280993
      w: 0.0167889135935 ]
  panda_link8 RPY = [-3.067797531765308, -0.09851913882123099, -0.7321389100283087]''', 0,1,1,board_origin, board_orientation)

  end_button_press = Coords('''joints = [0.412597410983 0.164181102811 -0.183924663199 -2.55672592441 0.0763932341404 2.7707845938 1.0086144719]
  panda_link8 pose = [
  header: 
  seq: 0
  stamp: 
    secs: 1682789626
    nsecs: 742530107
  frame_id: "/world"
  pose: 
  position: 
    x: 0.46298833408
    y: 0.0983415023025
    z: 0.21330782485
  orientation: 
    x: -0.911288604869
    y: 0.410895008805
    z: -0.0219114544515
    w: 0.0154356256343 ]
  panda_link8 RPY = [-3.0954165787477335, -0.027253823298962938, -0.7853981634]''', 0,1,1,board_origin, board_orientation)

  snapshot_new = Coords('''joints = [0.295419448452 0.317506556419 -0.177641662418 -2.21532600514 -0.00285269775413 2.49041714006 2.47392041014]
  panda_link8 pose = [
  header: 
    seq: 0
    stamp: 
      secs: 1683490536
      nsecs: 112688064
    frame_id: "/world"
  pose: 
    position: 
      x: 0.544487928928
      y: 0.0636153351285
      z: 0.231686842678
    orientation: 
      x: 0.387083943363
      y: -0.921412253781
      z: 0.0223457517211
      w: 0.0258098189191 ]
  panda_link8 RPY = [-3.120347979670299, -0.06490782456613771, -2.3468554756831264]
  ''',0,1,1,board_origin, board_orientation)

  return start_button_press, slider_snapshot, slider_push, slider_push_back, test_port_take_out, test_port_plug_in, hatch_grasped, hatch_lift_15, hatch_lift_30, hatch_lift_60, hatch_lift_75, hatch_lift_90, hatch_lift_100, hatch_lift_depart, cable_grab, cable_take_out, cable_plug_in_approach, cable_plug_in_prepare,end_button_press

###############################################

