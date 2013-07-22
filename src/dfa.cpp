#include "utils.h"

class automaton_c{
  public:

    set<label_c> labels;
    size_t S;
    size_t s0;
    map< pair<size_t, label_c>, set<size_t> > d;
    set<size_t> F;

    automaton_c() {}

    bool run(size_t s, size_t steps, label_c input[], bool& found_accepting_run){
      if(found_accepting_run){
        return true;
      }
      else if(steps == 0){
        //cout<<"s: "<< s <<" steps: "<< steps << endl; 
        if(F.find(s) != F.end())
          return true;
        else
          return false;
      }
      else
      {
        label_c& l = input[0];
        //cout<<"s: "<< s<<" steps: "<< steps <<" label: "<< (size_t)l.s << endl;
        assert(labels.find(l) != labels.end());
        auto& t1 = d[make_pair(s,l)];
        for(auto& sp : t1){
          if(run(sp, steps-1, &input[1], found_accepting_run))
          {
            found_accepting_run = true;
            break;
          }
        }
      }
    }

    void test(){
      srand(time(NULL));
      S = 2;
      s0 = 0;
      F.insert(0);
      label_c a(1);
      label_c b(2);
      labels.insert(a);
      labels.insert(b);

      d = { {{0,a},{1}}, {{0,b},{0}}, {{1,a},{1}}, {{1,b},{0}} };

      vector<label_c> input;
      for(size_t i=0; i<10; i++){
        auto p = next(labels.begin(), (int)(RANDF*labels.size()));
        input.push_back(*p);
        cout<< (size_t)(*p).s<<" ";
      }
      cout<<endl;
      bool res = false;
      run(s0, 10, &input[0], res);
      cout<<"accept: "<< res << endl;
    }
};

class dfa_c : public automaton_c{
  public:
    dfa_c(){
    }

    dfa_c operator*(const dfa_c dfa2){
      
      auto id = [=](int x, int y){return x*dfa2.S + y;};
      
      dfa_c dfa3;
      dfa3.S = S*dfa2.S;
      dfa3.s0 = id(s0,dfa2.s0);
      for(auto& s1 : F){
        for(auto& s2 : dfa2.F)
          dfa3.F.insert(id(s1,s2));
      }

      dfa3.labels = labels;
      for(auto& l : dfa2.labels)
        dfa3.labels.insert(l);
     
      auto& m3 = dfa3.d;
      for(auto& m1 : d){
        int s1 = m1.first.first;
        label_c l1 = m1.first.second;
        
        for(auto& s1e : m1.second)
        {
          for(auto& m2 : dfa2.d)
          {
            int s2 = m2.first.first;
            label_c l2 = m2.first.second;

            label_c l3 = l1.intersect(l2);
            auto key = make_pair(id(s1,s2), l3);
            for(auto& s2e : m2.second)
            {
              m3[key].insert(id(s1e,s2e));  
            }
          }
        }
      }
      return dfa3;
    }
};

class nfa_c : public automaton_c{
  public:
    nfa_c(){
    }
};

int main()
{
  dfa_c dfa;
  dfa.test();

  return 0;
}
