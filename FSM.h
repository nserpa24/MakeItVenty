// Code for MakeIt Venty
// by Nick Serpa, Jonathan Vail, Phil Martel
// License ??
/* Brief description:
 *  Header for Finite State Machine code
 */
#ifndef FSM_H
#define FSM_H
enum FSM_STATE {NO_STATE, STARTUP, READY, INHALE, EXHALE };

void FSM( void );
void SetFSMState( enum FSM_STATE s );
long TimeInFSMState( void );
#endif

