#ifndef MCL2__LIKELIHOODFIELD_HPP_
#define MCL2__LIKELIHOODFIELD_HPP_

namespace mcl
{
class LikelihoodField
{
public:
  LikelihoodField(double likelihood_dist);
  ~LikelihoodField();

  void initLikelihoodField();
};
}  // namespace mcl

#endif