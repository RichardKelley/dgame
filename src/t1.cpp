#include <iostream>
#include <inttypes.h>
#include <cstdlib>
#include <vector>
#include <set>
#include <map>
using namespace std;

#define RANDF   (rand()/(RAND_MAX+1.0))

typedef uint8_t symbol;

class label_c{
  public:
    const static int len = 8;
    symbol s;

    label_c() {};
    label_c(symbol s_) : s(s_){
    }
    void insert(label_c& l){
      s = s | l.s;
    }
    void remove(label_c& l){
      s = s ^ l.s;
    }
    void remove(const int w){
      s = s ^ (1<<w);
    }
    label_c minus(label_c& l){
      return label_c(s ^ l.s);
    }
    friend bool operator<(const label_c& a, const label_c& b){
      return a.s < b.s;
    }
    bool operator[](const int w){
      return (s & (1<<w));
    }
    bool operator!=(label_c& l){
      return (s != l.s);
    }
    void print(){
      for(int i=7; i>=0; i--)
        cout<< !!(s & (1<<i));
      cout<<endl;
    }
};

template<size_t dim>
class cost_c{
  public:
    vector<float> c;

    cost_c(){
      c = vector<float>(dim,0);
    }
    cost_c(vector<float>& c_) : c(c_) {}

    cost_c operator+(const cost_c& c2){
      cost_c toret;
      for(int i=0; i<dim; i++)
        toret += c2.c[i];
      return toret;
    }
    cost_c& operator+=(const cost_c& c2){
      for(int i=0; i<dim; i++)
        c[i] += c2.c[i];
      return *this;
    }
    bool operator<(const cost_c& c2){
      for(int i=0; i<dim; i++){
        if(c[i] < c2.c[i])
          return true;
        else if(c[i] > c2.c[i])
          return false;
      }
      return true;
    }
};

class automaton_c{
  public:
    int S;
    int s0;
    set<int> F;
    vector<label_c> labels;
    automaton_c() {}
    automaton_c(int s0_) : s0(s0_){};
};

class dfa_c : public automaton_c{
  public:
    typedef map<label_c, int>  transition;

    map<int, transition> d;
    dfa_c(){
    }
    
    bool run(int s, vector<label_c>& input, vector<int>& trace){
      trace.clear();
      trace.push_back(s);
      for(size_t i=0; i< input.size(); i++){
        label_c& l = input[i];
        transition t1 = d[s];
        s = t1[l];
        trace.push_back(s);
      }
      if(F.find(s) != F.end())
        return true;
      return false;
    }
    
    void test(){
      srand(time(NULL));
      S = 2;
      s0 = 0;
      F.insert(1);
      label_c a(1);
      label_c b(2);
      labels.push_back(a);
      labels.push_back(b);

      d = {{0,{{a,1}}}, {1,{{b,0}}}, {0,{{b,0}}}, {1,{{a,1}}}};
      
      vector<int> trace;
      vector<label_c> input;
      for(int i=0; i<10; i++){
        int p = RANDF*labels.size();
        input.push_back(labels[p]);
      }
      cout<<"accept: "<< run(s0, input, trace) << endl;
      for(size_t i=0; i< trace.size(); i++)
        cout<<trace[i]<<" ";
      cout<<endl;
    }
};

class nfa_c : public automaton_c{
  public:
    
    typedef map<label_c, set<int> > transition;
    map<int, transition> d;

    bool run(int s, int steps, label_c input[]){
      static int found_accepting_run = false;
      if(steps == 0){
        if(F.find(s) != F.end())
          return true;
        else
          return false;
      }
      else
      {
        label_c& l = input[0];
        transition& t1 = d[s];
        for(auto& sp : t1[l]){
          if(run(sp, steps-1, &input[1]))
            found_accepting_run = true;
        }
        return found_accepting_run;
      }
    }
    
    void test(){
      srand(time(NULL));
      S = 2;
      s0 = 0;
      F.insert(0);
      label_c a(1);
      label_c b(2);
      labels.push_back(a);
      labels.push_back(b);

      d = {{0,{{a,{1}}}}, {1,{{b,{0}}}}, {0,{{b,{0}}}}, {1,{{a,{1}}}}};
      
      label_c input[10];
      for(int i=0; i<10; i++){
        int p = RANDF*labels.size();
        input[i] = labels[p];
      }
      cout<<"accept: "<< run(s0, 10, &input[0]) << endl;
    }
};

int main()
{
  dfa_c dfa;
  dfa.test();

  nfa_c nfa;
  nfa.test();

  return 0;
}
