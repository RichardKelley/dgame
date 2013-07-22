#include "automaton.h"

int main()
{
  automaton_product<2> abar;
  automaton_ss_c psi(false, SIDEWALK, 1, 0);
  abar.insert(psi, 2);

  label_c l2(SIDEWALK);
  timed_word_c tw(1, l2);
  //tw.print();
  auto c = abar.get_cost(tw);
  c.print();
  return 0;
}
