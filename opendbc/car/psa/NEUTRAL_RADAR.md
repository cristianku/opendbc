# Prova locale ARTIV neutro — Peugeot 3008

Prima fase di emulazione, indipendente da planner, accelerazione richiesta,
lead, set speed e stato di engagement di openpilot. I payload sono fissi;
cambiano soltanto contatori, checksum e il bit alternato osservato in `0x2B6`.
Le richieste di coppia, frenata, prefill e ripartenza sono disattivate.
Non viene inviato `0x452` e non viene attivato il controllo longitudinale.

`CarController.update()` decide condizioni di invio, frequenze, contatori e
valori neutri della prova. In `psacan.py` ogni funzione codifica un solo
messaggio e riceve bus e tutti i segnali configurabili come argomenti
obbligatori, compreso il contatore dove presente. Il checksum è calcolato
dal packer. Le quattro funzioni riutilizzabili sono:

- `create_HS2_DYN1_MDD_ETAT_2B6`
- `create_HS2_DYN_MDD_ETAT_2F6`
- `create_HS2_DAT_ARTIV_V2_4F6`
- `create_HS2_SUPV_ARTIV_796`

`NeutralRadar` tiene lo stato della sessione e controlla le ricezioni; non
genera messaggi né gestisce le frequenze di invio.

## Payload di riferimento

| ID, bus 1 | Frequenza | DLC | Payload con contatore 0 |
| --- | ---: | ---: | --- |
| `0x2B6` | 50 Hz | 8 | `FE 00 00 02 00 00 03 0A` |
| `0x2F6` | 50 Hz | 8 | `00 FF 86 00 F8 60 0F 00` |
| `0x4F6` | 10 Hz | 5 | `FF FE 5F FE 00` |
| `0x796` | 1 Hz | 8 | `00 00 00 00 00 00 00 00` |

Fonte: route `00000039--186c45ae9d`, segmento 1, prima della richiesta
programming a +72,455 s dalla partenza della route, registrata con openpilot
`04bfc6d26` e opendbc `766a8903`. Auto ferma, freno premuto, ACC non attivo.
Il generatore implementato riproduce esattamente tutti i 533 campioni dei
quattro ID nella finestra analizzata da -5,0 a -0,2 s rispetto alla richiesta.

`ACC_STATUS=2` e `AUTO_BRAKING_STATUS=3` significano Inhibited nel DBC attuale.
I valori grezzi inattivi di decelerazione/coppia e i sentinella di distanza
sono conservati come registrati: non rappresentano richieste fisiche attive.
Il bit chiamato `GEAR_TYPE` nel DBC segue la parità del contatore in 720/720
campioni delle tre finestre controllate; il suo significato resta da verificare.
`0x4F6` replica lo stato sensore Active con nessun bersaglio. Questo non fornisce
le funzioni reali del radar o dell'AEB. La supervisione `0x796` riproduce la
condizione osservata, senza una validazione per ogni stato della vettura.

## Sequenza della prova

1. Solo Peugeot 3008: la richiesta programming esistente parte dopo 10 secondi
   consecutivi con auto ferma e CAN valido. Una sola richiesta per vita del controller.
2. Attesa di una risposta CAN effettiva `06 50 02 ...` da `0x696`, bus 1,
   successiva alla richiesta, e almeno 100 ms senza i quattro ID radar originali.
   Nessuna attivazione senza entrambi i riscontri entro un secondo.
   All'avvio dell'emulazione il CAN deve essere ancora valido; una risposta
   tardiva con CAN già invalido interrompe la prova senza inviare sostituti.
3. Invio dei quattro messaggi neutri alle frequenze indicate. Ogni secondo
   viene inviato `02 3E 00` a `0x6B6`, DLC 3, per mantenere la sessione.
   La risposta attesa è `02 7E 00`; il mantenimento va verificato sul veicolo.
4. Durante l'emulazione, soltanto gli echi TX radar effettivamente ricevuti
   dal Panda su src 129 vengono letti dal parser PSA come messaggi bus 1.
   I log originali non sono modificati; non vengono fabbricate ricezioni dai
   messaggi che il controller intende inviare. Gli invii rifiutati su src 193
   non soddisfano il parser. Non vengono disabilitati i controlli `canValid`.
5. Stop senza tentativi automatici se la vettura si muove, ritorna una
   trasmissione del radar originale, arriva un rifiuto diagnostico, manca
   traffico fisico sul bus ADAS per 250 ms, mancano gli echi TX o non arriva
   una risposta diagnostica per oltre 2 s. Gli echi TX hanno timeout di
   250 ms per `0x2B6`/`0x2F6`, 500 ms per `0x4F6` e 2 s per `0x796`.
   Dopo 250 ms iniziali per ricevere gli echi e allineare i contatori, anche
   CAN non valido su qualunque bus del veicolo interrompe la prova.

Dopo lo stop cessano sia i messaggi sostitutivi sia TesterPresent. Non viene
inviato alcun reset o comando di ripristino al radar. Nei log precedenti,
senza mantenimento, le trasmissioni originali riprendevano dopo circa 5,2 s;
il ritorno dopo questa nuova prova deve essere osservato.

Il ramo che normalmente inserisce richieste takeover in `0x2F6` è escluso
dal momento della richiesta ARTIV per la vita di questo controller, per non
mescolare dati di openpilot ai payload della prova. Le altre piattaforme PSA
conservano il comportamento precedente.

## Verifiche e prova sul veicolo

I test locali coprono byte di riferimento, frequenze, contatori, checksum,
assenza di richieste longitudinali, risposte diagnostiche, timeout, ritorno
del radar, arresto per movimento e indipendenza dai comandi di openpilot.
Un test usa la vera CarInterface: gli echi ricevuti mantengono `canValid`,
mentre la loro assenza produce nuovamente l'errore. La safety PSA esistente
consente i payload neutri sia con controlsAllowed falso sia vero.

La suite PSA contiene già un test fallito relativo al metodo commentato
`_update_cruise_button_events`; la modifica non interviene su quel codice.

Installazione e prova sul comma a cura di Cristian, con auto parcheggiata e
mantenuta ferma. Questa fase deve verificare sul veicolo se spariscono gli
errori di comunicazione, senza presumere che tutti gli avvisi ACC/AEB spariscano
con stati dichiarati Inhibited. Nei nuovi log controllare:

- risposta positiva programming, echi TX dei quattro ID e risposte TesterPresent;
- assenza degli stessi ID da una seconda sorgente radar sul bus 1;
- stato `canValid`, errori effettivi della vettura e loro evoluzione temporale;
- eventuale messaggio `ARTIV neutral: stopped (...)` e ritorno del radar originale.

Nessuna verifica sul veicolo o scomparsa degli errori è dimostrata dai soli test locali.
