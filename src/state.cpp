//#include <iostream>
#include <Arduino.h>
#include "state.h"

// ************************** concrete state instances **************************
char stopped_state_name[] = "stopped_state";
Stopped stopped_state(stopped_state_name);

char inactive_state_name[] = "inactive_state";
Inactive inactive_state(inactive_state_name);

char active_state_name[] = "active_state";
Active active_state(active_state_name);

char moving_forward_state_name[] = "moving_forward_state";
MovingForward moving_forward_state(moving_forward_state_name);

char moving_backward_state_name[] = "moving_backward_state";
MovingBackward moving_backward_state(moving_backward_state_name);

char moving_left_state_name[] = "moving_left_state";
MovingLeft moving_left_state(moving_left_state_name);

char moving_right_state_name[] = "moving_right_state";
MovingRight moving_right_state(moving_right_state_name);

char spinning_state_name[] = "spinning_state";
Spinning spinning_state(spinning_state_name);

// ************************** valid voice commands **************************
extern char NAME[];
extern char GO[];
extern char GO1[];
extern char STOP[];
extern char BACK[];
extern char LEFT[];
extern char RIGHT[];
extern char SPIN[];
extern char SPIN1[];
extern char OFF[];

// ************************** output state definitions **************************
extern const char motor1_;
extern const char motor2_;
extern const char motor3_;
extern const char motor4_;

extern const char green_led_;
extern const char red_led_;
extern const char forward_led_;
extern const char backward_led_;
extern const char left_led_;
extern const char right_led_;

int strlength(char *s) {
  int c = 0;
  while (*s != '\0') {
    c++;
    s++;
  }
  return c;
}

State* handle_transitions(char *event, State *curr_state) {
  if (strcmp(event, STOP) == 0) {
    return &stopped_state;
  } else if ((strcmp(event, GO) == 0) || (strcmp(event, GO1) == 0)) {
    return &moving_forward_state;
  } else if (strcmp(event, BACK) == 0) {
    return &moving_backward_state;
  } else if (strcmp(event, LEFT) == 0) {
    return &moving_left_state;
  } else if (strcmp(event, RIGHT) == 0) {
    return &moving_right_state;
  } else if ((strcmp(event, SPIN) == 0) || (strcmp(event, SPIN1) == 0)) {
    return &spinning_state;
  } else if (strcmp(event, OFF) == 0) {
    return &inactive_state;
  } else {
    return curr_state;
  }
}

State::State(char *name) {
  this->name = name;
}

void State::print_name(char *name) {
}

Inactive::Inactive(char *name) :
  State(name) {
}

State* Inactive::on_event(char *event) {
  if (strcmp(event, NAME) == 0) {
    return &active_state;
  } else {
    return this;
  }

}
void Inactive::perform_action(int *output_states) {
  //  cout << "Performing state: ";
  output_states[motor1_] = 0;
  output_states[motor2_] = 0;
  output_states[motor3_] = 0;
  output_states[motor4_] = 0;

  output_states[green_led_] = 0;
  output_states[red_led_] = 0;
  output_states[forward_led_] = 0;
  output_states[backward_led_] = 0;
  output_states[left_led_] = 0;
  output_states[right_led_] = 0;

}

// Concrete states
Stopped::Stopped(char *name) :
  State(name) {
}

State* Stopped::on_event(char *event) {

  return handle_transitions(event, this);

}
void Stopped::perform_action(int *output_states) {
  //  cout << "Performing state: ";
  output_states[motor1_] = 0;
  output_states[motor2_] = 0;
  output_states[motor3_] = 0;
  output_states[motor4_] = 0;

  output_states[green_led_] = 0;
  output_states[red_led_] = 1;
  output_states[forward_led_] = 0;
  output_states[backward_led_] = 0;
  output_states[left_led_] = 0;
  output_states[right_led_] = 0;

}

Active::Active(char *name) :
  State(name) {
}

State* Active::on_event(char *event) {
  return handle_transitions(event, this);

}
void Active::perform_action(int *output_states) {
  //  cout << "Performing state: ";
  //  print_name(this->name);
  output_states[motor1_] = 0;
  output_states[motor2_] = 0;
  output_states[motor3_] = 0;
  output_states[motor4_] = 0;

  output_states[green_led_] = 1;
  output_states[red_led_] = 0;
  output_states[forward_led_] = 0;
  output_states[backward_led_] = 0;
  output_states[left_led_] = 0;
  output_states[right_led_] = 0;

}

MovingForward::MovingForward(char *name) :
  State(name) {
}

State* MovingForward::on_event(char *event) {
  return handle_transitions(event, this);

}
void MovingForward::perform_action(int *output_states) {
  //  cout << "Performing state: ";
  //  print_name(this->name);
  output_states[motor1_] = 30;
  output_states[motor2_] = 30;
  output_states[motor3_] = 30;
  output_states[motor4_] = 30;

  output_states[green_led_] = 1;
  output_states[red_led_] = 0;
  output_states[forward_led_] = 1;
  output_states[backward_led_] = 0;
  output_states[left_led_] = 0;
  output_states[right_led_] = 0;

}

MovingBackward::MovingBackward(char *name) :
  State(name) {
}

State* MovingBackward::on_event(char *event) {
  return handle_transitions(event, this);

}
void MovingBackward::perform_action(int *output_states) {
  //  cout << "Performing state: ";
  //  print_name(this->name);
  output_states[motor1_] = -30;
  output_states[motor2_] = -30;
  output_states[motor3_] = -30;
  output_states[motor4_] = -30;

  output_states[green_led_] = 1;
  output_states[red_led_] = 0;
  output_states[forward_led_] = 0;
  output_states[backward_led_] = 1;
  output_states[left_led_] = 0;
  output_states[right_led_] = 0;
}

MovingLeft::MovingLeft(char *name) :
  State(name) {
}

State* MovingLeft::on_event(char *event) {
  return handle_transitions(event, this);

}
void MovingLeft::perform_action(int *output_states) {
  //  cout << "Performing state: ";
  //  print_name(this->name);
  output_states[motor1_] = +30;
  output_states[motor2_] = -30;
  output_states[motor3_] = +30;
  output_states[motor4_] = -30;

  output_states[green_led_] = 1;
  output_states[red_led_] = 0;
  output_states[forward_led_] = 0;
  output_states[backward_led_] = 0;
  output_states[left_led_] = 1;
  output_states[right_led_] = 0;
}

MovingRight::MovingRight(char *name) :
  State(name) {
}

State* MovingRight::on_event(char *event) {
  return handle_transitions(event, this);

}
void MovingRight::perform_action(int *output_states) {
  //  cout << "Performing state: ";
  //  print_name(this->name);
  output_states[motor1_] = -30;
  output_states[motor2_] = +30;
  output_states[motor3_] = -30;
  output_states[motor4_] = +30;

  output_states[green_led_] = 1;
  output_states[red_led_] = 0;
  output_states[forward_led_] = 0;
  output_states[backward_led_] = 0;
  output_states[left_led_] = 0;
  output_states[right_led_] = 1;
}

Spinning::Spinning(char *name) :
  State(name) {
}

State* Spinning::on_event(char *event) {
  return handle_transitions(event, this);

}
void Spinning::perform_action(int *output_states) {
  //  cout << "Performing state: ";
  //  print_name(this->name);
  output_states[motor1_] = -30;
  output_states[motor2_] = +30;
  output_states[motor3_] = +30;
  output_states[motor4_] = -30;

  output_states[green_led_] = 1;
  output_states[red_led_] = 0;
  output_states[forward_led_] = 1;
  output_states[backward_led_] = 1;
  output_states[left_led_] = 1;
  output_states[right_led_] = 1;

}
