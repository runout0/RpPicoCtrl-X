void PROGRAM0_init__(PROGRAM0 *data__, BOOL retain) {
  __INIT_LOCATED(BOOL,__IX0_8,data__->BTN1,retain)
  __INIT_LOCATED_VALUE(data__->BTN1,__BOOL_LITERAL(FALSE))
  __INIT_LOCATED(BOOL,__IX0_9,data__->BTN2,retain)
  __INIT_LOCATED_VALUE(data__->BTN2,__BOOL_LITERAL(FALSE))
  __INIT_LOCATED(BOOL,__QX0_0,data__->LED1GN,retain)
  __INIT_LOCATED_VALUE(data__->LED1GN,__BOOL_LITERAL(FALSE))
  __INIT_LOCATED(BOOL,__QX0_7,data__->RELAY4,retain)
  __INIT_LOCATED_VALUE(data__->RELAY4,__BOOL_LITERAL(FALSE))
  RS_init__(&data__->RS0,retain);
}

// Code part
void PROGRAM0_body__(PROGRAM0 *data__) {
  // Initialise TEMP variables

  __SET_VAR(data__->RS0.,S,,__GET_LOCATED(data__->BTN1,));
  __SET_VAR(data__->RS0.,R1,,__GET_LOCATED(data__->BTN2,));
  RS_body__(&data__->RS0);
  __SET_LOCATED(data__->,LED1GN,,__GET_VAR(data__->RS0.Q1,));
  __SET_LOCATED(data__->,RELAY4,,__GET_VAR(data__->RS0.Q1,));

  goto __end;

__end:
  return;
} // PROGRAM0_body__() 





