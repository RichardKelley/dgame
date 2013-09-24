#ifndef __automaton_h__
#define __automaton_h__

#include <smpl/smpl.h>

typedef uint8_t symbol;

class label_c{
  public:
    const static size_t len = 8;
    symbol s;

    label_c() : s(0){
    };
    label_c(const size_t w){
      s = (1<<w);
    }
    void insert(const size_t w){
      s = s | (1<<w);
    }
    void insert(const label_c& l){
      s = s | l.s;
    }
    void remove(const size_t w){
      if(!!(s & (1<< w)))
        s = s ^ (1<<w);
    }
    label_c intersect(const label_c& l) const{
      return label_c(s & l.s);
    }
    bool is_subset(const label_c& l) const{
      return !!(s & l.s);
    }
    friend bool operator<(const label_c& a, const label_c& b){
      return a.s < b.s;
    }
    bool operator[](const size_t w) const{
      return (s & (1<<w));
    }
    bool operator!=(label_c& l){
      return (s != l.s);
    }
    bool operator==(label_c& l){
      return s == l.s;
    }
    void print(){
      for(int i=7; i>=0; i--)
        cout<< !!(s & (1<<i));
      cout<<endl;
    }
};

class timed_word_c{
  public:
    float dt;
    label_c label;

    timed_word_c(float dt_, label_c label_) : dt(dt_), label(label_){
    }
    timed_word_c(const timed_word_c& tw){
      dt = tw.dt;
      label = tw.label;
    }
    void print(){
      cout<<"tw: "<< dt<<" ";
      label.print();
    }
};

class automaton_ss_c{
  public:
    bool is_positive;
    label_c label;
    float weight;
    float cost;
    
    automaton_ss_c(){}
    automaton_ss_c(bool is_positive_, const size_t w,
        float weight_, float cost_):
      is_positive(is_positive_), weight(weight_), cost(cost_){
        label = label_c(w);
      }

    float get_cost(const timed_word_c& tw) const{
      if( (label.is_subset(tw.label) && is_positive) ||
          ((!label.is_subset(tw.label)) && (!is_positive)))
        return 0;
      else
        return cost + weight*tw.dt;
    }

    friend bool operator<(const automaton_ss_c& psi1, const automaton_ss_c& psi2){
      return psi1.label < psi2.label;
    }
};

template<size_t max_priority>
class automaton_product_c{
  public:
    set<pair<size_t, automaton_ss_c> > rules;
  
    int insert(size_t p, const automaton_ss_c& psi){
      assert(p <= max_priority);
      auto pp = make_pair(p, psi);
      rules.insert(pp);
      return 0;
    };

    cost_c<max_priority+2> get_cost(const timed_word_c& tw){
      cost_c<max_priority+2> cost(0);
      for(auto& ppsi : rules)
      {
        auto& psi = ppsi.second;
        float t1 = psi.get_cost(tw);
        cost.val[ppsi.first] += t1; 
      }
      cost.val[max_priority+1] = tw.dt;
      return cost;
    };
};
#endif
