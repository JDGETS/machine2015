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
        virtual ~State() {};

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

      struct LinearStateMachineNode{
        State* state;
        LinearStateMachineNode* next;
      };

      class LinearStateMachine
      {
        //static const int NULL = 0;
        int count;
        LinearStateMachineNode* root;

      public:
        LinearStateMachine()
        { 
          count = 0;
          root = NULL;
        }

        ~LinearStateMachine()
        { 
          while(root != NULL)
          {
            LinearStateMachineNode* old = root;
            root = root->next;
            delete old;
          }
        }

        void Add(State* state)
        {
          LinearStateMachineNode* node = new LinearStateMachineNode;
          node->state = state;
          node->next = NULL;
          if(root == NULL)
            root = node;
          else{
            LinearStateMachineNode* last = root;
            while(last->next != NULL)
            {
              last = last->next;
            }
            last->next = node;
          }
          count++;
        }

        void Execute()
        {
          LinearStateMachineNode *node = root;
          while(node)
          {
            node->state->Start();
            node->state->Update();
            while(node->state->IsActive()){
              node->state->Execute();
              node->state->Update();
            }
            node = node->next;
          }
        }
      };
  }
}

#endif