#ifndef STATE_H
#define STATE_H

namespace Vehicle
{
  namespace States 
  {
      class State 
      {
      public:
        State() : _active(false) {};

        virtual void Start() 
        { 
          _active = true;
        };

        virtual void Update() 
        { 
          return;
        };

        virtual void Execute() = 0;

        const void End() 
        { 
          _active = false; 
        }
        
        const bool & IsActive() 
        { 
          return _active; 
        }

      private:
        bool _active;
      };
  }
}

#endif