#pragma once
#ifndef STATE_H
#define STATE_H

class State {
  public:
    char *name;
    State(char *name);
    void print_name(char *name);
    virtual State* on_event(char *event); // virtual method base definition is useless. All derived classes must implement this method.
    virtual void perform_action(int *output_states) = 0; // this is an interface now, no need for definition in cpp file.
};

class Stopped: public State {
  public:
    Stopped(char *name);
    State* on_event(char *event);
    void perform_action(int *output_states);
};

class Inactive: public State {
  public:
    Inactive(char *name);
    State* on_event(char *event);
    void perform_action(int *output_states);
};

class Active: public State {
  public:
    Active(char *name);
    State* on_event(char *event);
    void perform_action(int *output_states);
};

class MovingForward: public State {
  public:
    MovingForward(char *name);
    State* on_event(char *event);
    void perform_action(int *output_states);
};

class MovingBackward: public State {
  public:
    MovingBackward(char *name);
    State* on_event(char *event);
    void perform_action(int *output_states);
};

class MovingLeft: public State {
  public:
    MovingLeft(char *name);
    State* on_event(char *event);
    void perform_action(int *output_states);
};

class MovingRight: public State {
  public:
    MovingRight(char *name);
    State* on_event(char *event);
    void perform_action(int *output_states);
};

class Spinning: public State {
  public:
    Spinning(char *name);
    State* on_event(char *event);
    void perform_action(int *output_states);
};

#endif
