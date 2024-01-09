__includes["communicationManager.nls"  "missionManager.nls" "mvt-controller.nls"  "helper.nls"
           "obstacleManager.nls" "taskManager.nls" "formationManager.nls"]

globals [                      ;; section of the global variables
  goal-center-x  goal-center-y ;;  initial center of circle that englobes all destinations
  originX originY              ;;  intermediary center of circle that has all VN in its peripheral
  radius                       ;;  radius of circle that englobes all destinations
  nb-arrived                   ;; countes the number of arrived robots to their virtual nodes (VN)
  nb-dest-arrived              ;; countes the number of arrived VN to their intermediary/final target
  tot-flocks                   ;;  total number of elements of the flocks
  counter                      ;;  count the number of stages a mission has
  dest-coef-speed
  mission-list                 ;;  list of coordinates of the intermediate stages of the global mission of the robots
  triangle-vertices            ;;  list of coordinates of vertices of a triangle (if a triangle shape is used)
  x-lineUpPLeft y-lineUpPRight  x-lineDwnPLeft y-lineDwnPRight  ;; coordinates of the lines of the vertical and horizontal passages
  widthRect
  nb-messages
  orientation
]

breed [flocks     a-flock]  ;;   the group is call flocks, while the individual is called a-flock (any suggestion to change the name of the individual!!!!!!)
breed [destinations destination ]  ;;   the group is refered to as destinations, while the individual is called destination
breed [tokens token]

tokens-own [
  msg-from
  task-initiator
  msg-id
  msg
  task-data
]


flocks-own [        ;; agent (robot) structure
  new-heading       ;; new direction
  v-x      v-y      ;;  component of current velocity
  new-v-x  new-v-y      ;;  component of the calculated new velocity
  x-init  y-init
  to-xdeviation to-ydeviation
  my-destination     ;; determining a VN
  angle-to-destination
  collide-with
  nb-collision
  flock-id
  speed
  arrived           ;; reaching a VN
  has-destination   ;; did the agent find a VN
  reached-vn      ;; vn not yet reached = 0   reaching on going vn = 1  reaching vn in its goal = 2
  flock-mates  ;; the 'reference group' that the flock adjusts its movement to
  my-token
  received-message-id
  var-is-deviating
  start-slope-obs
  end-slope-obs
  nb-deviation
  deviation-from
  solving-rigide-formation
  free-move                  ;; moving freely with no formation
]


;; set of destinations targeted by the flocks
destinations-own [     ;; structure of the VN (Virtual Nodes)
  assigned-flock       ;; if an agent reached this VN
  status               ;;  free = 0  aasigned = 1  flock reaching vn = 2      vn reaching final target 3
  to-xpos   to-ypos    ;; target position of the VN to be reached by any robot
  v-x  v-y             ;; velocity of the VN on x and y
  speed
  dest-arrived         ;; true if the VN is in the target position
]


;; this simplifies the search by only considering agents (robots) on the current and neighbouring patches
patches-own [
  locale         ;; store the patch and its neighbours in a patch-set for efficiency
  local-flocks ;; flocks on the locale - to know the immediate neighbors in the adjascent patches

]


to init                                            ;; initialization of global variable defined in the global section
  set counter 1                                ;; number of intermediate steps of the flock in their travel
  set nb-arrived  0                            ;; to know the number of robots that reach their VN
  set nb-dest-arrived 0
  set dest-coef-speed 0.7
  set-default-shape flocks     "default"
  set-default-shape destinations "target"
  set nb-messages 0
  set message-id 0
  set orientation 1
end


to-report flock-in-place                    ;; determines/returns how many robots are around me in a distance of 3
   report count flocks in-radius 3

end

to flock-link-my-destination       ;; to create the link and show/hide them based on the user choice in the interface
  ask flocks [
    if my-destination != nobody [ create-link-with my-destination ]
  ]
end

to find-destination   ;; the methode is call in the initial phase. I did not let the robots compete to reach the VN.
  let nb-affect 0     ;; initially VN are assigne to robots. Robots have to reach this VN
  let nb-id-flocks 0
  foreach sort-on [flock-id] flocks [
    the-flock -> ask the-flock [
      set flock-id nb-id-flocks
      set nb-id-flocks nb-id-flocks + 1
      let ok false
      let term nobody
      ask min-one-of destinations with [status = 0] [who]  [
        set status  1
        ask myself [ set my-destination myself ]
        set ok true
        set nb-affect nb-affect + 1
      ]
      ask my-destination [set assigned-flock myself]
    ]
  ]

end


;; a robot updates stored set of local flocks for each patch
to update-local-flocks
  ask patches [
    set local-flocks flocks-on locale
  ]

end


to compute-locale
  ask patches [                     ;; color in white all patched
    set pcolor white
    set locale (patch-set self (patches in-radius ranges))  ;; for each patch assign to local (attribute defined above) all patches that are in the circle of radius ranges
  ]
end


;; ++++++++++++++++++++++++++++++++++  SETUP ++++++++++++++++++++++++++++++++++  This is the initial main the starting point and configuration of the program

to setup
  clear-all                          ;; clear every thing
  init                              ;; call the init method to initilize most of the global variable
  compute-locale

  let nbrejected-flocks 0          ;; as the robots are distributed randomly. two robots may be set in the same patches in which case they are rejected

  create-flocks ceiling nb-flocks  [     ;; creating the robots
    set xcor  random 50                  ;; random positions
    set ycor  random 50
    set x-init xcor
    set y-init ycor
    set color blue
    set nb-collision 0
    set arrived false
    set var-is-deviating false
    set size 2.5
    set reached-vn 0
    set has-destination false
    set received-message-id -1
    set solving-rigide-formation false
    set nb-deviation 0
    set free-move true
;    set label who
    let qw flock-in-place            ;; call to flock-in-place to check whether more than one robot in a given region defined in the method
    if  qw > 1  [                    ;; check the rejection of the created robots
      set nbrejected-flocks nbrejected-flocks + 1
      die                            ;; reject the robot that was created and that made this conflict
    ]
  ]
  set tot-flocks (nb-flocks - nbrejected-flocks)      ;; total number of the created robots, after the rejection phase

  create-all-tokens                 ;; create the tokens
  link-token-to-flock               ;; link each token to an agent

  create-destinations ceiling tot-flocks  [          ;; create the VN randomly in a specific region according to the number of robots
    set xcor  max-pxcor - 30 - random 30
    set ycor  max-pycor - 10 - random 20
    set status 0
    set dest-arrived false

    set color red
    set size 2.5                                 ;; determining the size of each VN
  ]

  set goal-center-x mean [xcor] of destinations ;; determining the center of gravity of the created VN
  set goal-center-y mean [ycor] of destinations
  set originX goal-center-x
  set originy goal-center-y
  mission
  ask destinations [                            ;; compute the raidus of the distribution
    let my-pos max-one-of destinations [ distancexy goal-center-x  goal-center-y ]   ;; the radius is the largest distance horizontally between VN
    ask my-pos [ set radius distancexy goal-center-x  goal-center-y ]
  ]

  init-circle                                   ;; create the initial circle to be reached by the VN
  find-destination                              ;; associate robots to VN
  update-local-flocks                           ;; determines initially the neighbours of each agents. This will be updated in each step (tick)
  build-Hpassage radius                         ;; create the H,V passages
  build-Vpassage (radius * 1.3)
  build-slope-obstacle1
;  build-slope-obstacle2
  reset-ticks                                   ;; initialize the ticks to zero
  ask flocks [update-flock-movement]
end



;; main    =================================== TO GO ==================    this is the infinite loop (ticks) it stops when it reaches the final goal
to go
  let iscol false
  let isobs false
  ask flocks [                                                            ;; update the flock mates: its neighbouring robots
    set flock-mates other local-flocks
  ]
  ask links [die]                                                         ;; delete links created in the previous steps

  ask flocks [ create-links-with flock-mates [ set color red + 3 ]  ]     ;; create links with its mate with a specific color
  if flock-destination-links = true [                                     ;; show link with VN if specified by the user in the interface
    ask flocks [ create-link-with my-destination ]
  ]
  ask flocks [                                                            ;; determine the mean of the speed of the mate include the robot itself
      set new-v-x mean [v-x] of (turtle-set self flock-mates)
      set new-v-y mean [v-y] of (turtle-set self flock-mates)

        ifelse (still-deviating or (nb-deviation > 0)) [
      forward  speed
    ]
    [
      set iscol is-colliding
      set isobs is-obstacling
      if (iscol = false  and isobs = false) [ forward speed]
    ]


    if reached-vn != 3 [
      if (iscol = false  and isobs = false) [
        update-flock-movement
      ]
    ]
  ]
  ask destinations [
    if dest-arrived = false [                                                    ;;  each VN has to compute its next move
      update-destination-movement
      let jump-dist distancexy to-xpos to-ypos                             ;; go to next destination provided that it will not be overtaken
      ifelse jump-dist > speed
       [ forward speed ]
      [
        ifelse status = 2 [forward speed]
        [forward jump-dist]
      ]
    ]
  ]

  update-local-flocks                                                     ;;

  ifelse (counter <= 8) and (nb-arrived = (counter * tot-flocks)) [        ;; the mission is of 7 steps if all robots reached their VN then go to next mission
    run-mission counter                                                     ;; call to runmission to perform mission 'counter'
    set counter counter + 1
  ]
  [
    if nb-arrived >= (8 * tot-flocks) [                                   ;; if all mission are performed stop
      show "======> all turtles arrived to their final destinations"
      ask destinations [set hidden? not hidden? ]
      stop
    ]
  ]
  tick
end
@#$#@#$#@
GRAPHICS-WINDOW
165
10
783
456
-1
-1
4.33
1
10
1
1
1
0
1
1
1
0
140
0
100
0
0
1
ticks
30.0

BUTTON
7
28
70
61
NIL
setup
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

SLIDER
4
102
147
135
density
density
0
2
0.25
0.05
1
NIL
HORIZONTAL

SLIDER
4
141
147
174
min-speed
min-speed
0
1
0.7
0.1
1
NIL
HORIZONTAL

SLIDER
4
181
142
214
max-speed
max-speed
min-speed
3
2.7
0.1
1
NIL
HORIZONTAL

SLIDER
4
225
144
258
ranges
ranges
preferred-distance
11
11.0
0.1
1
NIL
HORIZONTAL

BUTTON
77
29
140
62
step
go
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

SLIDER
991
57
1163
90
preferred-distance
preferred-distance
0
2
1.55
0.05
1
NIL
HORIZONTAL

SLIDER
991
98
1163
131
sectors-to-check
sectors-to-check
2
12
7.0
1
1
NIL
HORIZONTAL

SLIDER
992
141
1164
174
view-angle
view-angle
5
360
135.0
5
1
NIL
HORIZONTAL

BUTTON
75
67
138
100
NIL
go
T
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

SLIDER
992
15
1164
48
nb-flocks
nb-flocks
0
100
20.0
1
1
NIL
HORIZONTAL

SLIDER
990
184
1162
217
flocker-vision-dist
flocker-vision-dist
0
20
7.0
1
1
NIL
HORIZONTAL

SWITCH
4
301
156
334
flock-destination-links
flock-destination-links
1
1
-1000

SWITCH
5
260
145
293
hide-destination
hide-destination
1
1
-1000

SWITCH
1019
293
1167
326
elastic-formation
elastic-formation
1
1
-1000

@#$#@#$#@
## WHAT IS IT?

(a general understanding of what the model is trying to show or explain)

## HOW IT WORKS

(what rules the agents use to create the overall behavior of the model)

## HOW TO USE IT

(how to use the model, including a description of each of the items in the Interface tab)

## THINGS TO NOTICE

(suggested things for the user to notice while running the model)

## THINGS TO TRY

(suggested things for the user to try to do (move sliders, switches, etc.) with the model)

## EXTENDING THE MODEL

(suggested things to add or change in the Code tab to make the model more complicated, detailed, accurate, etc.)

## NETLOGO FEATURES

(interesting or unusual features of NetLogo that the model uses, particularly in the Code tab; or where workarounds were needed for missing features)

## RELATED MODELS

(models in the NetLogo Models Library and elsewhere which are of related interest)

## CREDITS AND REFERENCES

(a reference to the model's URL on the web if it has one, as well as any other necessary credits, citations, and links)
@#$#@#$#@
default
true
0
Polygon -7500403 true true 150 5 40 250 150 205 260 250

airplane
true
0
Polygon -7500403 true true 150 0 135 15 120 60 120 105 15 165 15 195 120 180 135 240 105 270 120 285 150 270 180 285 210 270 165 240 180 180 285 195 285 165 180 105 180 60 165 15

arrow
true
0
Polygon -7500403 true true 150 0 0 150 105 150 105 293 195 293 195 150 300 150

box
false
0
Polygon -7500403 true true 150 285 285 225 285 75 150 135
Polygon -7500403 true true 150 135 15 75 150 15 285 75
Polygon -7500403 true true 15 75 15 225 150 285 150 135
Line -16777216 false 150 285 150 135
Line -16777216 false 150 135 15 75
Line -16777216 false 150 135 285 75

bug
true
0
Circle -7500403 true true 96 182 108
Circle -7500403 true true 110 127 80
Circle -7500403 true true 110 75 80
Line -7500403 true 150 100 80 30
Line -7500403 true 150 100 220 30

butterfly
true
0
Polygon -7500403 true true 150 165 209 199 225 225 225 255 195 270 165 255 150 240
Polygon -7500403 true true 150 165 89 198 75 225 75 255 105 270 135 255 150 240
Polygon -7500403 true true 139 148 100 105 55 90 25 90 10 105 10 135 25 180 40 195 85 194 139 163
Polygon -7500403 true true 162 150 200 105 245 90 275 90 290 105 290 135 275 180 260 195 215 195 162 165
Polygon -16777216 true false 150 255 135 225 120 150 135 120 150 105 165 120 180 150 165 225
Circle -16777216 true false 135 90 30
Line -16777216 false 150 105 195 60
Line -16777216 false 150 105 105 60

car
false
0
Polygon -7500403 true true 300 180 279 164 261 144 240 135 226 132 213 106 203 84 185 63 159 50 135 50 75 60 0 150 0 165 0 225 300 225 300 180
Circle -16777216 true false 180 180 90
Circle -16777216 true false 30 180 90
Polygon -16777216 true false 162 80 132 78 134 135 209 135 194 105 189 96 180 89
Circle -7500403 true true 47 195 58
Circle -7500403 true true 195 195 58

circle
false
0
Circle -7500403 true true 0 0 300

circle 2
false
0
Circle -7500403 true true 0 0 300
Circle -16777216 true false 30 30 240

cow
false
0
Polygon -7500403 true true 200 193 197 249 179 249 177 196 166 187 140 189 93 191 78 179 72 211 49 209 48 181 37 149 25 120 25 89 45 72 103 84 179 75 198 76 252 64 272 81 293 103 285 121 255 121 242 118 224 167
Polygon -7500403 true true 73 210 86 251 62 249 48 208
Polygon -7500403 true true 25 114 16 195 9 204 23 213 25 200 39 123

cylinder
false
0
Circle -7500403 true true 0 0 300

dot
false
0
Circle -7500403 true true 90 90 120

face happy
false
0
Circle -7500403 true true 8 8 285
Circle -16777216 true false 60 75 60
Circle -16777216 true false 180 75 60
Polygon -16777216 true false 150 255 90 239 62 213 47 191 67 179 90 203 109 218 150 225 192 218 210 203 227 181 251 194 236 217 212 240

face neutral
false
0
Circle -7500403 true true 8 7 285
Circle -16777216 true false 60 75 60
Circle -16777216 true false 180 75 60
Rectangle -16777216 true false 60 195 240 225

face sad
false
0
Circle -7500403 true true 8 8 285
Circle -16777216 true false 60 75 60
Circle -16777216 true false 180 75 60
Polygon -16777216 true false 150 168 90 184 62 210 47 232 67 244 90 220 109 205 150 198 192 205 210 220 227 242 251 229 236 206 212 183

fish
false
0
Polygon -1 true false 44 131 21 87 15 86 0 120 15 150 0 180 13 214 20 212 45 166
Polygon -1 true false 135 195 119 235 95 218 76 210 46 204 60 165
Polygon -1 true false 75 45 83 77 71 103 86 114 166 78 135 60
Polygon -7500403 true true 30 136 151 77 226 81 280 119 292 146 292 160 287 170 270 195 195 210 151 212 30 166
Circle -16777216 true false 215 106 30

flag
false
0
Rectangle -7500403 true true 60 15 75 300
Polygon -7500403 true true 90 150 270 90 90 30
Line -7500403 true 75 135 90 135
Line -7500403 true 75 45 90 45

flower
false
0
Polygon -10899396 true false 135 120 165 165 180 210 180 240 150 300 165 300 195 240 195 195 165 135
Circle -7500403 true true 85 132 38
Circle -7500403 true true 130 147 38
Circle -7500403 true true 192 85 38
Circle -7500403 true true 85 40 38
Circle -7500403 true true 177 40 38
Circle -7500403 true true 177 132 38
Circle -7500403 true true 70 85 38
Circle -7500403 true true 130 25 38
Circle -7500403 true true 96 51 108
Circle -16777216 true false 113 68 74
Polygon -10899396 true false 189 233 219 188 249 173 279 188 234 218
Polygon -10899396 true false 180 255 150 210 105 210 75 240 135 240

house
false
0
Rectangle -7500403 true true 45 120 255 285
Rectangle -16777216 true false 120 210 180 285
Polygon -7500403 true true 15 120 150 15 285 120
Line -16777216 false 30 120 270 120

leaf
false
0
Polygon -7500403 true true 150 210 135 195 120 210 60 210 30 195 60 180 60 165 15 135 30 120 15 105 40 104 45 90 60 90 90 105 105 120 120 120 105 60 120 60 135 30 150 15 165 30 180 60 195 60 180 120 195 120 210 105 240 90 255 90 263 104 285 105 270 120 285 135 240 165 240 180 270 195 240 210 180 210 165 195
Polygon -7500403 true true 135 195 135 240 120 255 105 255 105 285 135 285 165 240 165 195

line
true
0
Line -7500403 true 150 0 150 300

line half
true
0
Line -7500403 true 150 0 150 150

pentagon
false
0
Polygon -7500403 true true 150 15 15 120 60 285 240 285 285 120

person
false
0
Circle -7500403 true true 110 5 80
Polygon -7500403 true true 105 90 120 195 90 285 105 300 135 300 150 225 165 300 195 300 210 285 180 195 195 90
Rectangle -7500403 true true 127 79 172 94
Polygon -7500403 true true 195 90 240 150 225 180 165 105
Polygon -7500403 true true 105 90 60 150 75 180 135 105

plant
false
0
Rectangle -7500403 true true 135 90 165 300
Polygon -7500403 true true 135 255 90 210 45 195 75 255 135 285
Polygon -7500403 true true 165 255 210 210 255 195 225 255 165 285
Polygon -7500403 true true 135 180 90 135 45 120 75 180 135 210
Polygon -7500403 true true 165 180 165 210 225 180 255 120 210 135
Polygon -7500403 true true 135 105 90 60 45 45 75 105 135 135
Polygon -7500403 true true 165 105 165 135 225 105 255 45 210 60
Polygon -7500403 true true 135 90 120 45 150 15 180 45 165 90

sheep
false
15
Circle -1 true true 203 65 88
Circle -1 true true 70 65 162
Circle -1 true true 150 105 120
Polygon -7500403 true false 218 120 240 165 255 165 278 120
Circle -7500403 true false 214 72 67
Rectangle -1 true true 164 223 179 298
Polygon -1 true true 45 285 30 285 30 240 15 195 45 210
Circle -1 true true 3 83 150
Rectangle -1 true true 65 221 80 296
Polygon -1 true true 195 285 210 285 210 240 240 210 195 210
Polygon -7500403 true false 276 85 285 105 302 99 294 83
Polygon -7500403 true false 219 85 210 105 193 99 201 83

square
false
0
Rectangle -7500403 true true 30 30 270 270

square 2
false
0
Rectangle -7500403 true true 30 30 270 270
Rectangle -16777216 true false 60 60 240 240

star
false
0
Polygon -7500403 true true 151 1 185 108 298 108 207 175 242 282 151 216 59 282 94 175 3 108 116 108

target
false
0
Circle -7500403 true true 0 0 300
Circle -16777216 true false 30 30 240
Circle -7500403 true true 60 60 180
Circle -16777216 true false 90 90 120
Circle -7500403 true true 120 120 60

tree
false
0
Circle -7500403 true true 118 3 94
Rectangle -6459832 true false 120 195 180 300
Circle -7500403 true true 65 21 108
Circle -7500403 true true 116 41 127
Circle -7500403 true true 45 90 120
Circle -7500403 true true 104 74 152

triangle
false
0
Polygon -7500403 true true 150 30 15 255 285 255

triangle 2
false
0
Polygon -7500403 true true 150 30 15 255 285 255
Polygon -16777216 true false 151 99 225 223 75 224

truck
false
0
Rectangle -7500403 true true 4 45 195 187
Polygon -7500403 true true 296 193 296 150 259 134 244 104 208 104 207 194
Rectangle -1 true false 195 60 195 105
Polygon -16777216 true false 238 112 252 141 219 141 218 112
Circle -16777216 true false 234 174 42
Rectangle -7500403 true true 181 185 214 194
Circle -16777216 true false 144 174 42
Circle -16777216 true false 24 174 42
Circle -7500403 false true 24 174 42
Circle -7500403 false true 144 174 42
Circle -7500403 false true 234 174 42

turtle
true
0
Polygon -10899396 true false 215 204 240 233 246 254 228 266 215 252 193 210
Polygon -10899396 true false 195 90 225 75 245 75 260 89 269 108 261 124 240 105 225 105 210 105
Polygon -10899396 true false 105 90 75 75 55 75 40 89 31 108 39 124 60 105 75 105 90 105
Polygon -10899396 true false 132 85 134 64 107 51 108 17 150 2 192 18 192 52 169 65 172 87
Polygon -10899396 true false 85 204 60 233 54 254 72 266 85 252 107 210
Polygon -7500403 true true 119 75 179 75 209 101 224 135 220 225 175 261 128 261 81 224 74 135 88 99

wheel
false
0
Circle -7500403 true true 3 3 294
Circle -16777216 true false 30 30 240
Line -7500403 true 150 285 150 15
Line -7500403 true 15 150 285 150
Circle -7500403 true true 120 120 60
Line -7500403 true 216 40 79 269
Line -7500403 true 40 84 269 221
Line -7500403 true 40 216 269 79
Line -7500403 true 84 40 221 269

wolf
false
0
Polygon -16777216 true false 253 133 245 131 245 133
Polygon -7500403 true true 2 194 13 197 30 191 38 193 38 205 20 226 20 257 27 265 38 266 40 260 31 253 31 230 60 206 68 198 75 209 66 228 65 243 82 261 84 268 100 267 103 261 77 239 79 231 100 207 98 196 119 201 143 202 160 195 166 210 172 213 173 238 167 251 160 248 154 265 169 264 178 247 186 240 198 260 200 271 217 271 219 262 207 258 195 230 192 198 210 184 227 164 242 144 259 145 284 151 277 141 293 140 299 134 297 127 273 119 270 105
Polygon -7500403 true true -1 195 14 180 36 166 40 153 53 140 82 131 134 133 159 126 188 115 227 108 236 102 238 98 268 86 269 92 281 87 269 103 269 113

x
false
0
Polygon -7500403 true true 270 75 225 30 30 225 75 270
Polygon -7500403 true true 30 75 75 30 270 225 225 270
@#$#@#$#@
NetLogo 6.2.0
@#$#@#$#@
@#$#@#$#@
@#$#@#$#@
@#$#@#$#@
@#$#@#$#@
default
0.0
-0.2 0 0.0 1.0
0.0 1 1.0 0.0
0.2 0 0.0 1.0
link direction
true
0
Line -7500403 true 150 150 90 180
Line -7500403 true 150 150 210 180
@#$#@#$#@
0
@#$#@#$#@
