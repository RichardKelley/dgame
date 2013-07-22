#include "automaton.h"

int main()
{
  automaton_product<2> abar;
  automaton_ss_c psi(false, SIDEWALK, 1, 0);
  abar.insert(psi, 2);
  return 0;
}
