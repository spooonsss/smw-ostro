; Ostro (from SMB2): a sprite that hops around, follows Mario, has a Shyguy driver, and can be picked up
; Credit to bigpotato for original graphics, Mellonpizza for Bob-omb disassembly

; Ostro comes with two options for graphics (SP4): "SMAS Ostro plus other SMB2.bin" and "bigpotato Ostro plus other SMB2.bin"

; "SMAS Ostro plus other SMB2.bin" graphics have 3 palette options:
;   Default SMW palette A
;   "SMAS Ostro - berry palette A.pal" overwrites just the last half of palette A, which is used when Yoshi tongues a pink berry
;   "SMAS Ostro - full palette A.pal" overwrites most of palette A, which changes the colors of the legs compared to "berry palette"

; "bigpotato Ostro plus other SMB2.bin" can be rendered with a pink palette or a purple palette
; "bigpotato Ostro purple-C pink-D.pal" uses the last half of palettes C and D, which are normally unused

; To enable carrying Ostro through a pipe, use "Carry Sprites Through Doors and Pipes" patch https://www.smwcentral.net/?p=section&a=details&id=20140

; Required configuration:

; Insert "SMB2 Shyguy + Giant Shyguy" https://www.smwcentral.net/?p=section&a=details&id=16800
; Set the following to the sprite number from list.txt
!driver_sprite_number = $44

; Optional configuration:

; Extra bit=1 => Ostro starts without a Shyguy driver

; Set to 0 to disable picking up Ostro
!enable_picking_up = 1
; Ostro faces Mario periodically
!turn_timer_frames = $3F
; When picked up, Ostro flails his legs periodically
!carried_leg_anim_frames = 8

; $08: red Shyguy palette
; $06: blue Shyguy palette
!driver_palette = $08
; This is the tile of Shyguy. Graphics page is controlled by Ostro config $166E (SP3/SP4 by default)
!driver_tile = $84

; Ostro doesn't face Mario when the driver has been picked up, by default
!turn_around_even_if_no_driver = 0

OstroTiles_head:
; SMAS Ostro graphics do not animate the head, so you could use CE for something else
; db $8E,$8E
db $8E,$CE
OstroTiles_body:
db $AE,$EE

OstroProps:
db $40,$00

; End of configuration


!sprite_anim = y_timer_offset_end-y_timer_offset-2

;; Defines stolen from Dyzen
!ButtonPressed_BYETUDLR = $15
!ButtonDown_BYETUDLR = $16
!ButtonPressed_AXLR0000 = $17
!ButtonDown_AXLR0000 = $18

!PlayerX = $94
!PlayerY = $96
!PlayerXSpeed = $7B
!PlayerYSpeed = $7D
!PowerUp = $19
!PlayerInAirFlag = $72
!PlayerDuckingFlag = $73
!PlayerClimbingFlag = $74
!PlayerDirection = $76
!PlayerRideYoshi = $187A|!Base2

!sprite_direction = !157C
!sprite_anim_frame = !1602
!sprite_turn_timer = !15AC

!sprite_anim_timer = !163E

!LockAnimationFlag = $9D

!OAM_XPos = $0300|!Base2
!OAM_YPos = $0301|!Base2
!OAM_Tile = $0302|!Base2
!OAM_Prop = $0303|!Base2

macro localJSL(dest, rtlop, db)
	; assert read1(<db><<16+<rtlop>) == $6B, "rtl op should point to a rtl"
	PHB			;first save our own DB
	PHK			;first form 24bit return address
	PEA.w ?return-1
	PEA.w <rtlop>-1		;second comes 16bit return address
	PEA.w (<db><<8)|(<db>)	;change db to desired value
	PLB
	PLB
	JML <dest>
?return:
	PLB			;restore our own DB
endmacro

macro FaceMario()
	%SubHorzPos()
	TYA
	STA !sprite_direction,x
endmacro

macro IsOnGround()
	LDA !sprite_blocked_status,x
	AND #$04
endmacro

macro IsTouchingCeiling()
	LDA !sprite_blocked_status,x
	AND #$08
endmacro

print "INIT ",pc
	; No driver if extra bit is set
	LDA !extra_bits,x
	AND #$04
	BNE +

	LDA #1
	STA !C2,X
+
	%FaceMario()
	LDA.b #!sprite_anim
	STA !sprite_anim_timer,x

	RTL

Ostro_SpeedX:
	;db $0C,$F4
	db $10,-$10

;;;;;;;;;;;;;;
;;			;;
;;	MAIN	;;
;;			;;
;;;;;;;;;;;;;;

print "MAIN ",pc
	;; Handle which state to manage
	phb : phk : plb
	jsr status_dispatch
	plb : rtl


status_dispatch:
	LDA !sprite_status,x
	JSL $0086DF|!BankB ; execute_pointer
	dw status0_empty
	dw status1_init
	dw status2_dead
	dw status3_smushed
	dw status4_spinjumped
	dw status5_sinking_in_lava
	dw status6_goal_tape_coin
	dw status7_in_yoshi_mouth
	dw status8_Ostro_Main
	dw status9_Ostro_Stunned
	dw statusab_HandleSprCarried
	dw statusab_HandleSprCarried


status0_empty:
status1_init:
status3_smushed:
status7_in_yoshi_mouth:
	RTS ; should be unreachable

status4_spinjumped:
	%localJSL($019A52|!BankB, $9D66, $01|(!BankB>>16)) ; HandleSprSpinJump
	RTS

status6_goal_tape_coin:
	JSL $00FBAC|!BankB ; LvlEndSprCoins
	RTS

status5_sinking_in_lava:
	JSR Ostro_Draw
	LDA !LockAnimationFlag
	BEQ +
	RTS
+
; based on HandleSprLava ($019A7B) Routine to handle a sprite killed by lava (sprite status 5).
	LDA.w !1558,X ; lava timer
	BNE +
	STZ.w !sprite_status,X ; timer ran out; erase sprite
	LDY.w !161A,X
	LDA.b #$00
	STA.w !1938,Y ; don't respawn
	RTS
+
	LDA.b #$04 ; Sinking Y speed.
	STA !AA,X
	ASL.w !190F,X               ; |\ Ignore walls when moving
	LSR.w !190F,X               ; |/
	LDA !B6,X                   ; |\
	BEQ .after_speed_adjustment ; || Slow down the sprite horizontally.
	BPL .rightward_horiz_speed  ; ||
	INC !B6,X                   ; ||
	BRA +                       ; ||
.rightward_horiz_speed:         ; ||
	DEC !B6,X                   ; |/
+
	LDA.w !1588,X
	AND.b #$03                  ; | Sprite is blocked from left or right
	BEQ .after_speed_adjustment ; |\ Clear X speed if it hits a block
	STZ !B6,X                   ; |/
.after_speed_adjustment:        ; |
	LDA.b #$01                  ; |\ Send the sprite behind objects.
	STA.w !1632,X               ; |/

	BRA +
status2_dead:
	JSR Ostro_Draw
	lda !LockAnimationFlag
	beq +
	rts
+

	LDA #$00
	%SubOffScreen()
	JSL $01802A|!BankB ; SubUpdateSprPos update position with gravity

	RTS


status8_Ostro_Main:
	JSR Ostro_Draw

	LDA !LockAnimationFlag
	BEQ +
	RTS

+

	%IsOnGround()
	BEQ .AfterGround

	; set anim_frame if anim_timer has expired
	LDA !sprite_anim_timer,x
	BNE +
	LDA !sprite_anim_frame,x
	EOR #1
	STA !sprite_anim_frame,x
	LDA.b #!sprite_anim
	STA !sprite_anim_timer,x
+

	; set X speed
	LDY !sprite_direction,x
	LDA Ostro_SpeedX,y
	EOR !sprite_slope,x
	ASL A
	LDA Ostro_SpeedX,y
	BCC +
	CLC
	ADC !sprite_slope,x
	+
	STA !sprite_speed_x,x
.AfterGround
	; clear X speed if touching object in direction of motion
	LDY !sprite_direction,x
	TYA
	INC A
	AND !sprite_blocked_status,x
	AND #$03
	BEQ +
	STZ !sprite_speed_x,x
	+
	LDA #$00
	%SubOffScreen()

	JSL $01802A|!BankB ; update position

	%IsOnGround()
	BEQ .SpriteInAir

	; Yspeedthings
	LDA !sprite_blocked_status,x
	BMI ++
	LDA #$00
	LDY !sprite_slope,x
	BEQ +
	++
	LDA #$18
+
	STA !sprite_speed_y,x

	; Flip sprite direction if touching block from side
	LDA !sprite_direction,x
	INC A
	AND !sprite_blocked_status,x
	AND #$03
	BEQ +
	JSR FlipSpriteDir
+

	; Turn around when timer is set as needed
	LDA !sprite_turn_timer,x
	BNE +
if !turn_around_even_if_no_driver = 0
	; don't turn around if there is no driver
	LDA !C2,x
	BEQ +
endif
	%FaceMario()

.SpriteInAir
	LDA #!turn_timer_frames
	STA !sprite_turn_timer,x
+

	JSL $018032|!BankB ; SprSprInteract

	; add y_timer_offset to sprite position
	; we do this only for Mario interaction so that interaction with blocks/slopes/other sprites is consistent
	LDY !sprite_anim_timer,x
	LDA !D8,x
	PHA
	CLC
	ADC y_timer_offset,y
	STA !D8,x

	LDA y_timer_offset,y
	TAY
	LDA !14D4,x
	PHA
	ADC #0
	CPY #0
	BPL +
	DEC A
+
	STA !14D4,x

    JSL $03B664|!BankB ; GetMarioClipping
    JSL $03B69F|!BankB ; GetSpriteClippingA

	; make the hitbox taller for mario
	!heighten_hitbox = 6
	LDA $05
	SEC
	SBC #!heighten_hitbox
	STA $05
	LDA $0B
	SBC #0
	STA $0B

	LDA $07
	CLC
	ADC #!heighten_hitbox
	STA $07

    JSL $03B72B|!BankB ; CheckForContact
	BCS +

.return2
	PLA
	STA !14D4,x
	PLA
	STA !D8,x

	RTS
+

	LDA $96 ; low byte of SubVertPos
	SEC
	SBC !D8,x
	CMP #$DC

	BMI .riding

.sprite_wins
	PLA
	STA !14D4,x
	PLA
	STA !D8,x

	LDA $1490|!addr ; branch if Mario has a star
	BNE .StarDeath

	LDA !PlayerRideYoshi ; return if on yoshi
	BNE +

	JSL $00F5B7|!bank ; hurt mario
+
	RTS

	.StarDeath
	%Star() ; star death
	RTS

.riding
	LDA $7D                 ; if mario speed is upward, return
	BMI .return2

	LDA #$01                ; set "on sprite" flag
	STA $1471|!Base2
	LDA #$06                ; disable interactions for a few frames
	STA !154C,x
	STZ $7D                 ; mario Y speed = 0
	; place mario above sprite
	LDA #$D4
	LDY !PlayerRideYoshi
	BEQ .no_yoshi
	LDA #$C4
.no_yoshi:
	CLC
	ADC !D8,x
	STA $96
	LDA !14D4,x
	ADC #$FF
	STA $97

	LDA !sprite_direction,x
	INC A
	AND $77			; load player blocked status flags
	AND #$03		; check if blocked on sides
	BNE .MaybePickUp	;if yes, don't update X-pos.

	; don't update mario's x position if he is at the right/left edge and we are moving right/left
	; this prevents mario from cycling between on/off the sprite when our x speed isn't a constant whole pixels (i.e. when !sprite_speed_x & $F != 0)
    LDA $94 ; SubHorzPos but just low byte
    SEC
    SBC !E4,x
	LDY !sprite_direction,x
	CMP max_riding_separation,y
	BEQ .MaybePickUp

.update_mario_pos
	LDY #$00
	LDA $1491|!Base2 ; move mario along with us
	BPL +
	DEY
+
	CLC
	ADC $94
	STA $94
	TYA
	ADC $95
	STA $95

.MaybePickUp
	; Test for X/Y button press
	LDA $16
	AND #$40
	BEQ .return

	; branch if carrying an enemy, or on yoshi
	LDA $1470|!Base2
	ORA !PlayerRideYoshi
	BNE .return

	; does ostro have a driver
if !enable_picking_up != 0
	LDA !C2,X
	BEQ .carry_me
else
	LDA !C2,X
	BEQ .return
endif

	STZ $00
	LDA #-$10
	STA $01
	STZ $02
	STZ $03

	LDA #!driver_sprite_number
	SEC ; spawn custom sprite
	; we need to call the driver's init ourselves, since we need its sprite state = $B
	; (sprite init is run normally when state = 1)
	; so we save the init routine pointer from pixi
	JSL SpawnSpriteSaveInit

	BCS .return

	TYX

	LDA #2
	STA !extra_byte_1,x ; set Shyguy to use palette in 15F6

	LDA !15F6,y
	AND #%11110001
	ORA #!driver_palette
	STA !15F6,y

	; call sprite init routine
	PHY

	PHK
	PEA .after_init-1
	JML [$08|!Base1]
.after_init:
	PLY

	LDA #$0B		; sprite status = Carried
	STA !14C8,y
	LDA #$FF		; set time until recovery
	STA !1540,y
	;LDA #$7F
	;STA !1564,y

	LDX $15E9|!Base2 ; restore ostro sprite index

	STZ !C2,X ; ostro doesn't have a driver anymore

	PLA
	STA !14D4,x
	PLA
	STA !D8,x

	RTS

.carry_me
	; Set sprite status to being carried and set mario to carry an item
	LDA #$0B
	STA !sprite_status,x
	INC $1470|!Base2

	; Set pose to hold an item
	LDA #$08
	STA $1498|!Base2

	; set animation timer
	LDA.b #!carried_leg_anim_frames
	STA !sprite_anim_timer,x
	; change animation frame
	LDA !sprite_anim_frame,x
	EOR #1
	STA !sprite_anim_frame,x

.return
	PLA
	STA !14D4,x
	PLA
	STA !D8,x
	RTS


; aka thrown
status9_Ostro_Stunned:
	JSR Ostro_Draw

	%SubOffScreen()

	; Update sprite position with gravity
	JSL $01802A|!BankB

	; Interact with sprites.
	JSL $018032|!BankB

	; Ostro runs toward Mario after hitting the ground
	%IsOnGround()
	BEQ +
	LDA #$08
	STA !sprite_status,x
	%FaceMario()
+

	%IsTouchingCeiling()
	BEQ .Not_Touching_Ceiling

	LDA #$10
	STA !sprite_speed_y,x

 .Not_Touching_Ceiling
	; turn around if we hit a wall horizontally
	LDA !sprite_direction,x
	INC A
	AND !sprite_blocked_status,x
	AND #$03
	BEQ +
	JSR FlipSpriteDir
+

.return
	RTS


FlipSpriteDir:
	; LDA !sprite_turn_timer,x
	; BNE .return
	LDA #!turn_timer_frames
	STA !sprite_turn_timer,x
	LDA !sprite_speed_x,x
	EOR #$FF
	INC A
	STA !sprite_speed_x,x
	LDA !sprite_direction,x
	EOR #$01
	STA !sprite_direction,x
.return
	RTS


statusab_HandleSprCarried:
	; nearly exactly the bob-omb disassembly for this state
	;; Handles all the logic for carrying sprites
	jsr Carried_Sprite_Main

	;; If player is in a way that the sprite needs to be drawn infront of them,
	;; set OAM index to zero.
	lda $13DD|!Base2 : bne +
	lda $1419|!Base2 : bne +
	lda $1499|!Base2 : beq ++
	+
	stz !sprite_oam_index,x
	++
	;; If signaled that player is to be drawn behind layers, set draw priority to #$10.
	ldy $1419|!Base2 : beq +
	jmp Ostro_Draw_Behind
	+
	jmp Ostro_Draw

Carried_Sprite_Main:
	;; call object interaction routine
	jsl $019138|!BankB

	;; Go back to stunned state if the player has a special animation,
	;; and yoshi isn't set to have a special value for drawing behind pipes
	lda $71 : cmp #$01 : bcc +
	lda $1419|!Base2 : bne +
	lda #$09 : sta !sprite_status,x
	rts
	+
	;; Return if the sprite has changed back into its normal state
	lda !sprite_status,x : cmp #$08 : bne +
	rts
	+
	lda !LockAnimationFlag : beq +
	jmp Attatch_Sprite_To_Player
+
	jsl $018032|!BankB ; SprSprInteract

	; animate the legs
	LDA !sprite_anim_timer,x
	BNE +
	LDA !sprite_anim_frame,x
	EOR #1
	STA !sprite_anim_frame,x
	LDA.b #!carried_leg_anim_frames
	STA !sprite_anim_timer,x
+

	;; Check if X/Y not held; if not, then release sprite
	lda $1419|!Base2 : bne +
	bit !ButtonPressed_BYETUDLR
	bvc ReleaseSprCarried
	+
	jmp Attatch_Sprite_To_Player

ReleaseSprCarried:
	;; Clear number of enemies killed.
	stz !1626,x

	;; Clear Y speed and set to stunned state.
	stz !sprite_speed_y,x
	lda #$09 : sta !sprite_status,x

	;; Branch to throw the sprite upward
	lda !ButtonPressed_BYETUDLR : and.B #$08 : bne TossUpSprCarried

	;; Branch to kick sprite left/right
	lda !ButtonPressed_BYETUDLR : and #$03 : bne KickSprCarried
	+
	;; Else, the sprite is to be dropped down.
	ldy !PlayerDirection
	lda $D1 : clc : adc Drop_Xoffset_Low,y : sta !sprite_x_low,x
	lda $D2 : adc Drop_Xoffset_High,y : sta !sprite_x_high,x
	%SubHorzPos()
	lda Drop_Xspeed,y : clc : adc !PlayerXSpeed : sta !sprite_speed_x,x
	bra StartKickPose

Drop_Xspeed:
db $FC,$04

Drop_Xoffset_Low:
db $F3,$0D
Drop_Xoffset_High:
db $FF,$00

TossUpSprCarried:
	;; Display contact graphic
	jsl $01AB6F|!BankB

	;; Set sprite speeds (-112 Y, Player/2 X)
	lda #$90 : sta !sprite_speed_y,x
	lda !PlayerXSpeed : sta !sprite_speed_x,x
	asl a : ror !sprite_speed_x,x
	bra StartKickPose

KickSprCarried:
	;; Display contact graphic
	jsl $01AB6F|!BankB

	;; Set sprite X speed
	ldy !PlayerDirection : lda !PlayerRideYoshi : beq +
	iny #2
	+
	lda ShellSpeedX,y : sta !sprite_speed_x,x
	eor !PlayerXSpeed : bmi StartKickPose
	lda !PlayerXSpeed : sta $00
	asl $00 : ror
	clc : adc ShellSpeedX,y : sta !sprite_speed_x,x

StartKickPose:
	;; Disable collisions with mario for 16 frames
	LDA #$10 : sta !154C,x

	;; Display kicking pose
	lda #$0C : sta $149A|!Base2
	rts

ShellSpeedX:
db $D2,$2E,$CC,$34

Attatch_Sprite_To_Player:
	;; Get index to table which will determine where in relation to the player,
	;; on the x axis the sprite will be moved to
	LDY #$00
	LDA !PlayerDirection
	STA !sprite_direction,x
	bne +
	iny
	+
	lda $1499|!Base2 : beq +
	iny #2
	cmp #$05 : bcc +
	iny
	+
	;; if mario is facing the screen or climbing, use the final index
	lda $1419|!Base2 : beq +
	cmp #$02 : beq ++
	+
	lda $13DD|!Base2 : ora !PlayerClimbingFlag : beq +
	++
	ldy #$05
	+
	;; If the player is on a sprite that calculate's the player's position based
	;; on the current frame, then use $94-$97 to calculate the carried sprite's
	;; position. otherwise use $D1-D4. This should ensure the sprite never looks
	;; "disjointed" in relation to the player
	phy : ldy #$00
	lda $1471|!Base2 : cmp #$03 : beq +
	ldy #$3D
	+
	;; Store player positions to scratch ram
	lda $0094|!Base1,y : sta $00
	lda $0095|!Base1,y : sta $01
	lda $0096|!Base1,y : sta $02
	lda $0097|!Base1,y : sta $03
	ply
	lda $00 : clc : adc CarriedSpr_OffsetToPlayer_Low,y : sta !sprite_x_low,x
	lda $01 : adc CarriedSpr_OffsetToPlayer_High,y : sta !sprite_x_high,x

	lda #$0D : ldy !PlayerDuckingFlag : bne +
	ldy !PowerUp : bne ++
	+
	lda #$0F
	++
	ldy $1489|!Base2 : beq +
	lda #$0F
	+
	clc : adc $02 : sta !sprite_y_low,x
	lda $03 : adc #$00 : sta !sprite_y_high,x

	LDA !sprite_y_low,x
	SEC
	SBC #$10
	STA !sprite_y_low,x
	LDA !sprite_y_high,x
	SBC #$00
	STA !sprite_y_high,x

	;; Set flags to indicate player is holding an item
	lda #$01 : sta $148F|!Base2 : sta $1470|!Base2
	rts

CarriedSpr_OffsetToPlayer_Low:
db $0B,$F5,$04,$FC,$04,$00

CarriedSpr_OffsetToPlayer_High:
db $00,$FF,$00,$FF,$00,$00



Ostro_Draw_Behind:
	LDA #$10
	BRA +
Ostro_Draw:
	LDA #$20
	+

	LDY !sprite_direction,x
	ORA OstroProps,y
	ORA !sprite_oam_properties,x
	STA $02

	%GetDrawInfo()
	PHX

	LDA !sprite_status,x
	CMP #$08
	BNE .draw_ostro

	PHY
	LDY !sprite_anim_timer,x
	INY
	LDA y_timer_offset,y
	PLY
	CLC
	ADC $01
	STA $01

	LDA !C2,X
	BEQ .draw_ostro

.draw_driver
	LDA !sprite_direction,x
	TAX
	LDA DriverOffsets,x
	CLC
	ADC $00
	STA !OAM_XPos,y

	LDA $01
	SEC
	SBC #$10-1
	STA !OAM_YPos,y

	LDA #!driver_tile
	STA !OAM_Tile,y

	LDA $02
	; set palette YXPPCCCT
	AND #%11110001
	ORA #!driver_palette
	STA !OAM_Prop,y

	INY #4

	LDX $15E9|!Base2 ; current sprite index

.draw_ostro
	LDA $00
	STA !OAM_XPos,y
	STA !OAM_XPos+4,y

	LDA $01
	STA !OAM_YPos+4,y
	SEC
	SBC #$10
	STA !OAM_YPos,y
	LDA !sprite_anim_frame,x
	TAX
	LDA OstroTiles_head,x
	STA !OAM_Tile,y
	LDA OstroTiles_body,x
	STA !OAM_Tile+4,y
	LDA $02
	STA !OAM_Prop,y
	STA !OAM_Prop+4,y
	PLX

	; y-flip if dead/carried
	LDA !sprite_status,x
	CMP #$08
	BEQ +
	LDA $02
	ORA #$80
	STA !OAM_Prop,y
	STA !OAM_Prop+4,y
	; swap head/body y position
	LDA !OAM_YPos,y
	PHA
	LDA !OAM_YPos+4,y
	STA !OAM_YPos,y
	PLA
	STA !OAM_YPos+4,y
+

	LDA #$02
	CLC
	ADC !C2,X
	LDY #$02
	%FinishOAMWrite()

	RTS


DriverOffsets: db -7, 7

y_timer_offset:
; needs to begin and end with 0. untested with increment/decrement > 1
db 0, -1, -2, -3, -4, -4, -4, -5, -5, -5, -5, -4, -4, -4, -3, -2, -1, 0
;db 0, 0, -1, -1, -1, -1, -1, -1, -2, -2, -2, -2, -2, -2, -3, -3, -3, -3, -3, -3, -3, -3, -3, -2, -2, -2, -2, -2, -1, -1, -1, 0, 0, 0
;db 0, 0, 0, 0, 0
y_timer_offset_end:

max_riding_separation:
db $C,-$C

assert !sprite_anim >= 2
assert !sprite_anim < 255

macro SpawnSpriteSaveInit()
global SpawnSpriteSaveInit:
incsrc "routines/SpawnSpriteSaveInit.asm"
endmacro

%SpawnSpriteSaveInit()
