;==============================================================================
; UNIVERSIDAD DEL CAUCA - INGENIERIA ELECTRONICA Y TELECOMUNICACIONES
; PRACTICA 1 - MICROCONTROLADORES (ENSAMBLADOR)
;
; Sistema de supervision de temperatura con PIC18F4550
;   - Sensor LM35 leido por el ADC (canal AN0)
;   - Conversion ADC -> grados Celsius con el factor EXACTO para LM35
;     (10mV/C) con Vref=5V y ADC de 10 bits: T(C) = ADC * 125 / 256
;     (no T(C)=ADC/2, que es solo una aproximacion rapida y se desvia
;      cada vez mas a medida que sube la temperatura)
;   - Timer0 genera el instante periodico de muestreo (~1.05 s)
;   - INT0 (RB0): alterna la alarma manual (LED en RA1)
;   - INT1 (RB1): alterna el ventilador (RA2)
;   - INT2 (RB2): alterna la unidad mostrada (Celsius / Fahrenheit)
;   - Alarma AUTOMATICA: al llegar a 30 grados Celsius (equivalen a 86 F;
;     como tempC es el valor base del que sale tempF, comparar contra 30 en
;     Celsius ya cubre los dos casos sin importar la unidad que este mostrando
;     el display) se encienden solos el LED de alarma (RA1) y el ventilador
;     (RA2); al bajar de nuevo de ese umbral se apagan solos. Esto se evalua
;     solo en el INSTANTE en que la temperatura cruza el umbral (no en cada
;     lectura), para que los pulsadores manuales (RB0/RB1) manden por encima
;     de la automatica: si el usuario apaga manualmente el LED o el
;     ventilador mientras la temperatura sigue >=30, se quedan apagados
;     hasta el siguiente cruce del umbral.
;   - 2 displays de 7 segmentos SENCILLOS, de CATODO COMUN, sin multiplexar:
;       PORTD (RD0-RD6) = digito de UNIDADES (a,b,c,d,e,f,g = bit0..bit6, directo)
;       PORTC + PORTE = digito de DECENAS, con dos detalles importantes:
;         1) en el PIC18F4550 de 40 pines (DIP, protoboard) el pin RC3 NO
;            EXISTE fisicamente (ahi esta VUSB), asi que el segmento 'd' de
;            este display NO se conecta en RC3 sino en RC7 (que si esta libre).
;         2) los segmentos e y f de este display NO se conectan en RC4/RC5:
;            se comprobo con el multimetro que esos dos pines del PIC no
;            funcionan como salida (solo como entrada digital), asi que e y
;            f se movieron a RE0 y RE1:
;           a=RC0  b=RC1  c=RC2  d=RC7  e=RE0  f=RE1  g=RC6   (RC3,RC4,RC5 sin usar)
;     (el pin comun de cada display va a GND; cada segmento se enciende
;      poniendo en 1 el pin del PORT correspondiente)
;
; Oscilador: INTERNO a 4 MHz (no se usa cristal externo)
;
;==============================================================================

#include <xc.inc>

; xc.inc ya usa el nombre "BOR" como macro para un bit de RCON (bandera de
; Brown-out Reset). Lo deshabilitamos aqui para poder usar "BOR" como nombre
; de ajuste en la directiva CONFIG, sin que el preprocesador lo reemplace.
#undef  BOR

;------------------------------------------------------------------------------
; BITS DE CONFIGURACION
;------------------------------------------------------------------------------
    CONFIG  FOSC    = INTOSCIO_EC   ; Oscilador interno, RA6 libre como E/S (no se usa cristal)
    CONFIG  FCMEN   = OFF           ; Sin monitor de fallo de reloj (no aplica, no hay reloj externo)
    CONFIG  IESO    = OFF           ; Sin arranque en dos etapas (no aplica sin oscilador externo)

    ; El PLL y el modulo USB no se usan en esta practica; se dejan en su
    ; configuracion mas simple para que la palabra de configuracion quede
    ; completamente definida (sin advertencias de "valor por defecto").
    CONFIG  PLLDIV  = 1             ; Sin preescalado del PLL (el PLL no se usa)
    CONFIG  CPUDIV  = OSC1_PLL2     ; Sin post-escalado del reloj de CPU
    CONFIG  USBDIV  = 1             ; Reloj USB tomado directo del oscilador (USB no se usa)

    CONFIG  PWRT    = ON            ; Power-up Timer activado (arranque mas estable)
    CONFIG  BOR     = ON            ; Reset por bajo voltaje activado (proteccion basica)
    CONFIG  VREGEN  = OFF           ; Regulador USB apagado (no se usa el modulo USB)

    CONFIG  WDT     = OFF           ; Perro guardian APAGADO (para no complicar el codigo con CLRWDT)

    CONFIG  PBADEN  = OFF           ; MUY IMPORTANTE: RB0-RB4 arrancan como DIGITALES, no analogicos
    CONFIG  MCLRE   = ON            ; Pin MCLR funciona como reset externo (con pull-up externo a Vcc)

    CONFIG  LVP     = OFF           ; Programacion en bajo voltaje apagada (libera RB5)
    CONFIG  XINST   = OFF           ; Juego de instrucciones extendido APAGADO (modo simple/clasico)
    CONFIG  STVREN  = ON            ; Reset si la pila se desborda (ayuda a detectar errores)
    CONFIG  DEBUG   = OFF

    CONFIG  CP0 = OFF, CP1 = OFF, CP2 = OFF, CP3 = OFF
    CONFIG  CPB = OFF, CPD = OFF
    CONFIG  WRT0 = OFF, WRT1 = OFF, WRT2 = OFF, WRT3 = OFF
    CONFIG  WRTB = OFF, WRTC = OFF, WRTD = OFF
    CONFIG  EBTR0 = OFF, EBTR1 = OFF, EBTR2 = OFF, EBTR3 = OFF, EBTRB = OFF

;------------------------------------------------------------------------------
; VARIABLES (memoria RAM de proposito general, banco de acceso)
;------------------------------------------------------------------------------
    PSECT   udata_acs
W_TEMP:                 DS  1       ; guarda W durante la interrupcion
STATUS_TEMP:            DS  1       ; guarda STATUS durante la interrupcion
BSR_TEMP:                DS  1       ; guarda BSR durante la interrupcion

flagUnidad:             DS  1       ; 0 = mostrar Celsius , 1 = mostrar Fahrenheit
flagDatoListo:          DS  1       ; se pone en 1 cuando el ADC ya tiene un dato nuevo
flagActualizar:         DS  1       ; se pone en 1 cuando hay que refrescar el display sin dato nuevo
flagAlarmaAuto:         DS  1       ; 1 = la alarma automatica (>=50 C) esta activa en este momento

adcRawL:                DS  1       ; copia del resultado del ADC (ADRESL)
tempC:                  DS  1       ; temperatura calculada en grados Celsius (0-99)
tempF:                  DS  1       ; temperatura calculada en grados Fahrenheit (saturada a 99)

valorMostrar:           DS  1       ; valor (0-99) que se va a separar en decenas/unidades
decenas:                DS  1       ; digito de las decenas a mostrar
unidades:                DS  1       ; digito de las unidades a mostrar
digitoTmp:               DS  1       ; usado dentro de convertir_7seg
patronDec:               DS  1       ; patron de 7 segmentos ya reacomodado para PORTC (decenas)
patronE:                 DS  1       ; segmentos e,f de las DECENAS, para PORTE (bit0=e, bit1=f)

multL:                   DS  1       ; parte baja de la multiplicacion (Celsius -> Fahrenheit)
multH:                   DS  1       ; parte alta de la multiplicacion (Celsius -> Fahrenheit)
restoDiv:                 DS  1       ; usado en las divisiones por resta sucesiva

;------------------------------------------------------------------------------
; VECTOR DE RESET (direccion absoluta 0x0000)
;------------------------------------------------------------------------------
    PSECT   resetVec,class=CODE,abs,ovrld
    ORG     0x0000
resetVec:
    goto    inicio

;------------------------------------------------------------------------------
; VECTOR DE INTERRUPCION DE ALTA PRIORIDAD (unico vector usado, IPEN=0)
;------------------------------------------------------------------------------
    PSECT   intVec,class=CODE,abs,ovrld
    ORG     0x0008
intVec:
    goto    isr_alta

;==============================================================================
; PROGRAMA PRINCIPAL
;==============================================================================
    PSECT   code,class=CODE
inicio:
    ;--- Reloj interno a 4 MHz --------------------------------------------
    movlw   01100000B          ; IRCF2:IRCF0 = 110 (4 MHz), SCS = 00
    movwf   OSCCON,a

    ;--- Estado inicial de displays: apagados ------------------------------
    ; En catodo comun, un segmento se enciende con un 1. Para apagar los
    ; dos displays al arrancar, simplemente ponemos los puertos en 0.
    clrf    PORTC,a
    clrf    PORTD,a
    clrf    PORTA,a
    clrf    PORTB,a
    clrf    PORTE,a

    ;--- Direcciones de los pines (TRIS) -----------------------------------
    movlw   11111001B          ; RA0 = entrada (AN0), RA1,RA2 = salidas, resto sin usar (entradas)
    movwf   TRISA,a
    movlw   00000111B          ; RB0,RB1,RB2 = entradas (INT0,INT1,INT2)
    movwf   TRISB,a
    clrf    TRISC,a             ; PORTC completo como salida (display decenas: a,b,c,d,g)
    clrf    TRISD,a             ; PORTD completo como salida (display unidades)
    clrf    TRISE,a             ; RE0,RE1 como salida (segmentos e,f de las decenas)

    ;--- Modulo ADC ----------------------------------------------------------
    movlw   00000001B          ; ADON=1, GO/DONE=0, canal CHS3:CHS0=0000 (AN0)
    movwf   ADCON0,a
    movlw   00001110B          ; VCFG=00 (Vdd/Vss), PCFG3:PCFG0=1110 -> solo AN0 analogico
    movwf   ADCON1,a
    movlw   10111011B          ; ADFM=1 (justificado a la derecha), ACQT=111 (20 TAD, sin CPU),
    movwf   ADCON2,a            ; ADCS=011 (reloj FRC, independiente del oscilador principal)

    ;--- Timer0: 16 bits, prescaler 1:16, reloj interno, sin precarga -------
    movlw   10000011B          ; TMR0ON=1, T08BIT=0(16 bits), T0CS=0(interno), PSA=0, T0PS=011(1:16)
    movwf   T0CON,a

    ;--- Interrupciones externas (flanco de bajada, pull-ups activados) -----
    bcf     RBPU                ; activa resistencias de pull-up internas de PORTB
    bcf     INTEDG0              ; INT0 dispara en flanco de bajada
    bcf     INTEDG1              ; INT1 dispara en flanco de bajada
    bcf     INTEDG2              ; INT2 dispara en flanco de bajada

    bcf     TMR0IF
    bcf     INT0IF
    bcf     INT1IF
    bcf     INT2IF
    bcf     ADIF

    bsf     TMR0IE
    bsf     INT0IE
    bsf     INT1IE
    bsf     INT2IE
    bsf     ADIE

    ;--- Variables iniciales ---------------------------------------------
    clrf    flagUnidad,a          ; arranca mostrando Celsius
    clrf    flagDatoListo,a
    clrf    flagActualizar,a
    clrf    flagAlarmaAuto,a      ; arranca sin la alarma automatica activa
    clrf    tempC,a
    clrf    tempF,a

    ;--- Habilitar interrupciones (al final, cuando todo ya esta listo) ----
    bsf     PEIE
    bsf     GIE

;------------------------------------------------------------------------------
; BUCLE PRINCIPAL
; No se consulta el estado de RB0/RB1/RB2 aqui (esta prohibido por la guia).
; Solo se revisan banderas de software que las interrupciones van dejando listas.
;------------------------------------------------------------------------------
main_loop:
    btfsc   flagDatoListo,0,a
    call    nueva_lectura

    btfsc   flagActualizar,0,a
    call    refrescar_display

    goto    main_loop

;==============================================================================
; SUBRUTINAS DEL PROGRAMA PRINCIPAL
;==============================================================================

;--- Se ejecuta cuando el ADC entrego un dato nuevo (bandera flagDatoListo) ---
nueva_lectura:
    bcf     flagDatoListo,0,a

    ; T(C) = ADC * 125 / 256  (formula exacta, ver subrutina convertir_ADC_a_C)
    call    convertir_ADC_a_C

    ; El display solo tiene 2 digitos: si tempC paso de 99, lo dejamos en 99
    movlw   100
    subwf   tempC,W,a          ; W = tempC - 100
    btfss   CARRY               ; si CARRY=1, tempC es >= 100 (hay que saturar)
    goto    fin_sat_C
    movlw   99
    movwf   tempC,a
fin_sat_C:

    call    revisar_alarma_auto
    call    refrescar_display
    return

;--- Alarma automatica: enciende/apaga solos el LED (RA1) y el ventilador ----
; (RA2) al CRUZAR el umbral de 30 grados Celsius (para arriba o para abajo).
; Se compara siempre contra tempC (el valor base, ya calculado antes de
; separarlo en Celsius/Fahrenheit para el display), asi que el umbral es el
; mismo sin importar que unidad este mostrando el display en ese momento.
; Solo actua en el instante del cruce (usando flagAlarmaAuto para recordar
; si ya estaba activa o no): asi, si el usuario despues apaga manualmente el
; LED o el ventilador con los pulsadores mientras la temperatura se queda
; arriba de 30, la automatica no lo vuelve a encender en la siguiente
; lectura -- el pulsador manual manda hasta el proximo cruce del umbral.
revisar_alarma_auto:
    movlw   30
    subwf   tempC,W,a           ; W = tempC - 30
    btfss   CARRY                ; CARRY=1 si tempC >= 30
    goto    temp_bajo_umbral

    ; tempC >= 30 -----------------------------------------------------------
    btfsc   flagAlarmaAuto,0,a  ; ya estaba activa?
    return                        ; si, no se repite la accion (pudo apagarla el usuario)
    bsf     flagAlarmaAuto,0,a
    bsf     PORTA,1,a             ; enciende LED de alarma (RA1)
    bsf     PORTA,2,a             ; enciende ventilador (RA2)
    return

temp_bajo_umbral:
    ; tempC < 30 --------------------------------------------------------------
    btfss   flagAlarmaAuto,0,a  ; ya estaba inactiva?
    return                        ; si, no se repite la accion
    bcf     flagAlarmaAuto,0,a
    bcf     PORTA,1,a             ; apaga LED de alarma (RA1)
    bcf     PORTA,2,a             ; apaga ventilador (RA2)
    return

;--- Convierte el valor crudo del ADC (adcRawL) a grados Celsius -------------
; El LM35 entrega 10mV por cada grado, y el ADC (10 bits, Vref=5V) entrega
; un codigo de 0 a 1023 proporcional al voltaje de entrada. Con esos datos,
; la formula exacta para pasar de codigo ADC a grados Celsius es:
;
;     T(C) = ADC * 5000mV / 1024 / 10mV  =  ADC * 125 / 256
;
; (para temperaturas de 0 a 99C el resultado del ADC siempre cabe en el
;  byte bajo ADRESL, por eso "adcRawL" de 8 bits alcanza sin problema)
;
; ADC*125 se calcula con corrimientos y sumas (nada de instrucciones de
; multiplicacion): se usa la misma idea que ya usa calcular_F mas abajo
; para multiplicar por 9, solo que aqui hay que sumar mas terminos porque
; 125 = 64+32+16+8+4+1. Dividir entre 256 despues es gratis: el resultado
; de la multiplicacion queda en 16 bits (multH:multL) y basta con quedarse
; con el byte alto (multH).
;
; Quedarse con el byte alto sin mas trunca siempre hacia abajo (por
; ejemplo 25.9 queda en 25, nunca sube a 26), y como el propio ADC ya
; trunca antes al convertir el voltaje a un numero entero, ese doble
; truncamiento hace que el resultado final quede sistematicamente 1 grado
; POR DEBAJO del valor real. Por eso antes de quedarnos con el byte alto
; se suma 128 (la mitad de 256) al byte bajo: eso es lo mismo que redondear
; al entero mas cercano en vez de truncar siempre hacia abajo.
convertir_ADC_a_C:
    clrf    multL,a
    clrf    multH,a               ; acumulador de la suma, arranca en 0

    movf    adcRawL,W,a
    movwf   restoDiv,a            ; restoDiv:digitoTmp = copia de adcRawL, en 16 bits
    clrf    digitoTmp,a           ; (restoDiv = byte bajo, digitoTmp = byte alto)

    ; --- sumar adcRawL*1 (bit0 de 125 = 1) ---
    movf    restoDiv,W,a
    addwf   multL,F,a
    movf    digitoTmp,W,a
    btfsc   CARRY
    addlw   1
    addwf   multH,F,a

    ; --- duplicar a *2 (bit1 de 125 = 0, no se suma) ---
    bcf     CARRY
    rlcf    restoDiv,F,a
    rlcf    digitoTmp,F,a

    ; --- duplicar a *4 y sumar (bit2 de 125 = 1) ---
    bcf     CARRY
    rlcf    restoDiv,F,a
    rlcf    digitoTmp,F,a
    movf    restoDiv,W,a
    addwf   multL,F,a
    movf    digitoTmp,W,a
    btfsc   CARRY
    addlw   1
    addwf   multH,F,a

    ; --- duplicar a *8 y sumar (bit3 de 125 = 1) ---
    bcf     CARRY
    rlcf    restoDiv,F,a
    rlcf    digitoTmp,F,a
    movf    restoDiv,W,a
    addwf   multL,F,a
    movf    digitoTmp,W,a
    btfsc   CARRY
    addlw   1
    addwf   multH,F,a

    ; --- duplicar a *16 y sumar (bit4 de 125 = 1) ---
    bcf     CARRY
    rlcf    restoDiv,F,a
    rlcf    digitoTmp,F,a
    movf    restoDiv,W,a
    addwf   multL,F,a
    movf    digitoTmp,W,a
    btfsc   CARRY
    addlw   1
    addwf   multH,F,a

    ; --- duplicar a *32 y sumar (bit5 de 125 = 1) ---
    bcf     CARRY
    rlcf    restoDiv,F,a
    rlcf    digitoTmp,F,a
    movf    restoDiv,W,a
    addwf   multL,F,a
    movf    digitoTmp,W,a
    btfsc   CARRY
    addlw   1
    addwf   multH,F,a

    ; --- duplicar a *64 y sumar (bit6 de 125 = 1) ---
    bcf     CARRY
    rlcf    restoDiv,F,a
    rlcf    digitoTmp,F,a
    movf    restoDiv,W,a
    addwf   multL,F,a
    movf    digitoTmp,W,a
    btfsc   CARRY
    addlw   1
    addwf   multH,F,a

    ; multH:multL = adcRawL * 125
    ; redondeo: sumar 128 al byte bajo (con su acarreo al byte alto) antes
    ; de dividir entre 256, para redondear al entero mas cercano en vez de
    ; truncar siempre hacia abajo
    movlw   128
    addwf   multL,F,a
    btfsc   CARRY
    incf    multH,F,a

    ; dividir entre 256 = quedarse con el byte alto
    movf    multH,W,a
    movwf   tempC,a
    return

;--- Redibuja el display con el ultimo dato disponible (nuevo o no) ----------
refrescar_display:
    bcf     flagActualizar,0,a

    btfsc   flagUnidad,0,a
    goto    usar_F
    movf    tempC,W,a
    movwf   valorMostrar,a
    goto    separar_digitos
usar_F:
    call    calcular_F
    movf    tempF,W,a
    movwf   valorMostrar,a

separar_digitos:
    call    dividir_entre_10

    movf    decenas,W,a
    call    convertir_7seg          ; W = patron a,b,c,d,e,f,g para las DECENAS
    movwf   patronDec,a             ; guardamos una copia: hace falta para sacar e,f y para remapear d

    ; --- Segmentos e (bit4) y f (bit5) de las DECENAS van por PORTE ---
    clrf    patronE,a
    btfsc   patronDec,4,a           ; segmento e encendido?
    bsf     patronE,0,a             ; -> RE0
    btfsc   patronDec,5,a           ; segmento f encendido?
    bsf     patronE,1,a             ; -> RE1
    movf    patronE,W,a
    movwf   PORTE,a

    ; --- Resto de segmentos de las DECENAS (a,b,c,d,g) van por PORTC ---
    movf    patronDec,W,a
    call    remapear_decenas    ; mueve 'd' de bit3 a bit7 (RC7) y apaga los bits que no van por PORTC
    movwf   PORTC,a

    movf    unidades,W,a
    call    convertir_7seg
    movwf   PORTD,a
    return

;--- Reacomoda el patron de 7 segmentos SOLO para el display de las DECENAS --
; Dos ajustes sobre el patron que entrega convertir_7seg, antes de mandarlo
; a PORTC (ver nota al inicio del archivo):
;   1) el segmento 'd' (bit 3) hay que pasarlo al bit 7 (RC7), porque el pin
;      RC3 no existe fisicamente en el PIC18F4550 de 40 pines.
;   2) los segmentos 'e' (bit 4) y 'f' (bit 5) ya NO se manejan por PORTC
;      (se comprobo que RC4/RC5 no sirven como salida): se apagan aqui
;      porque ese trabajo ahora lo hace PORTE (ver separar_digitos).
remapear_decenas:
    movwf   patronDec,a
    bcf     patronDec,7,a
    btfss   patronDec,3,a       ; el segmento 'd' (bit3) esta encendido?
    goto    fin_remap_d
    bsf     patronDec,7,a       ; si estaba encendido, se enciende en bit7 (RC7)
fin_remap_d:
    bcf     patronDec,3,a       ; bit3 (RC3) siempre apagado, ese pin no existe
    bcf     patronDec,4,a       ; bit4 (e) ya no va por PORTC, ahora va por RE0
    bcf     patronDec,5,a       ; bit5 (f) ya no va por PORTC, ahora va por RE1
    movf    patronDec,W,a
    return

;--- Convierte Celsius (tempC) a Fahrenheit (tempF), con saturacion a 99 -----
; Formula exacta: F = (C * 9 / 5) + 32
; Multiplicacion por 9 = (C*8) + C, usando corrimientos (sin instruccion MUL)
; Division entre 5 = restas sucesivas (sin instruccion DIV)
calcular_F:
    movf    tempC,W,a
    movwf   multL,a
    clrf    multH,a

    bcf     CARRY
    rlcf    multL,F,a
    rlcf    multH,F,a            ; multH:multL = C*2

    bcf     CARRY
    rlcf    multL,F,a
    rlcf    multH,F,a            ; multH:multL = C*4

    bcf     CARRY
    rlcf    multL,F,a
    rlcf    multH,F,a            ; multH:multL = C*8

    movf    tempC,W,a
    addwf   multL,F,a
    btfsc   CARRY
    incf    multH,F,a            ; multH:multL = C*8 + C = C*9

    clrf    tempF,a                ; aqui se va formando el cociente (C*9/5)
divF_loop:
    movf    multH,W,a
    bnz     divF_resta           ; si la parte alta no es 0, seguro que queda >= 5
    movlw   5
    subwf   multL,W,a
    btfss   CARRY                ; si multL < 5 (C=0), ya terminamos de dividir
    goto    divF_fin
divF_resta:
    movlw   5
    subwf   multL,F,a
    btfss   CARRY
    decf    multH,F,a
    incf    tempF,F,a
    goto    divF_loop
divF_fin:
    movlw   32
    addwf   tempF,F,a              ; tempF = (C*9/5) + 32

    ; El display solo tiene 2 digitos: si tempF paso de 99, lo dejamos en 99
    movlw   100
    subwf   tempF,W,a          ; W = tempF - 100
    btfss   CARRY               ; si CARRY=1, tempF es >= 100 (hay que saturar)
    goto    fin_sat_F
    movlw   99
    movwf   tempF,a
fin_sat_F:
    return

;--- Separa "valorMostrar" (0-99) en decenas y unidades, por restas sucesivas -
dividir_entre_10:
    clrf    decenas,a
    movf    valorMostrar,W,a
    movwf   restoDiv,a
div10_loop:
    movlw   10
    subwf   restoDiv,W,a
    btfss   CARRY
    goto    div10_fin
    movwf   restoDiv,a
    incf    decenas,F,a
    goto    div10_loop
div10_fin:
    movf    restoDiv,W,a
    movwf   unidades,a
    return

;--- Convierte un digito (0-9, en W) al patron de 7 segmentos --------------
; Displays de CATODO COMUN: 1 = segmento encendido, 0 = segmento apagado
; Se compara el digito contra 0,1,2... uno por uno.
;   bit:      7  6  5  4  3  2  1  0
;   segmento: -  g  f  e  d  c  b  a
convertir_7seg:
    movwf   digitoTmp,a

    movlw   0
    subwf   digitoTmp,W,a
    btfsc   ZERO
    retlw   00111111B           ; 0  -> a,b,c,d,e,f
    movlw   1
    subwf   digitoTmp,W,a
    btfsc   ZERO
    retlw   00000110B           ; 1  -> b,c
    movlw   2
    subwf   digitoTmp,W,a
    btfsc   ZERO
    retlw   01011011B           ; 2  -> a,b,d,e,g
    movlw   3
    subwf   digitoTmp,W,a
    btfsc   ZERO
    retlw   01001111B           ; 3  -> a,b,c,d,g
    movlw   4
    subwf   digitoTmp,W,a
    btfsc   ZERO
    retlw   01100110B           ; 4  -> b,c,f,g
    movlw   5
    subwf   digitoTmp,W,a
    btfsc   ZERO
    retlw   01101101B           ; 5  -> a,c,d,f,g
    movlw   6
    subwf   digitoTmp,W,a
    btfsc   ZERO
    retlw   01111101B           ; 6  -> a,c,d,e,f,g
    movlw   7
    subwf   digitoTmp,W,a
    btfsc   ZERO
    retlw   00000111B           ; 7  -> a,b,c
    movlw   8
    subwf   digitoTmp,W,a
    btfsc   ZERO
    retlw   01111111B           ; 8  -> a,b,c,d,e,f,g
    retlw   01101111B           ; 9  -> a,b,c,d,f,g (unico valor que falta)

;==============================================================================
; RUTINA DE INTERRUPCION (unico vector, IPEN=0)
;==============================================================================
isr_alta:
    ; Guardamos W, STATUS y BSR para no danar lo que el programa principal
    ; estaba haciendo antes de entrar aqui. Como W ya quedo a salvo en
    ; W_TEMP, podemos usar W libremente para copiar los otros dos registros.
    movwf   W_TEMP,a

    movf    STATUS,W,a
    movwf   STATUS_TEMP,a

    movf    BSR,W,a
    movwf   BSR_TEMP,a

    btfsc   TMR0IF
    call    atender_tmr0

    btfsc   INT0IF
    call    atender_int0

    btfsc   INT1IF
    call    atender_int1

    btfsc   INT2IF
    call    atender_int2

    btfsc   ADIF
    call    atender_adc

    ; Restauramos todo en el orden contrario al que se guardo
    movf    BSR_TEMP,W,a
    movwf   BSR,a
    movf    STATUS_TEMP,W,a
    movwf   STATUS,a
    movf    W_TEMP,W,a
    retfie  0

;--- Timer0: cada ~1.05 s se dispara una nueva conversion del ADC ------------
atender_tmr0:
    bcf     TMR0IF
    bsf     GO_DONE
    return

;--- INT0 (RB0): alterna la alarma manual (LED en RA1) -----------------------
atender_int0:
    bcf     INT0IF
    movlw   00000010B           ; bit 1 = RA1 (LED de alarma)
    xorwf   PORTA,F,a
    return

;--- INT1 (RB1): alterna el ventilador (RA2) ---------------------------------
atender_int1:
    bcf     INT1IF
    movlw   00000100B           ; bit 2 = RA2 (ventilador)
    xorwf   PORTA,F,a
    return

;--- INT2 (RB2): alterna la unidad mostrada (Celsius/Fahrenheit) -------------
atender_int2:
    bcf     INT2IF
    movlw   1
    xorwf   flagUnidad,F,a
    bsf     flagActualizar,0,a
    return

;--- ADC: guarda el resultado y avisa al programa principal ------------------
atender_adc:
    bcf     ADIF
    movf    ADRESL,W,a
    movwf   adcRawL,a
    bsf     flagDatoListo,0,a
    return

    END     resetVec
