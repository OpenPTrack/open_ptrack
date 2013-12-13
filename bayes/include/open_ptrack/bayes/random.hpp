/*
 * Bayes++ the Bayesian Filtering Library
 * Copyright (c) 2002 Michael Stevens
 * See accompanying Bayes++.htm for terms and conditions of use.
 *
 * $Id: random.hpp 634 2010-08-15 16:39:44Z mistevens $
 */

/*
 * Good random numbers from Boost
 *  Provides a common class  for all random number requirements to test Bayes++
 */

#include <boost/version.hpp>
#include <boost/random.hpp>


namespace Bayesian_filter
{

namespace
{
	// ISSUE variate_generator cannot be used without Partial template specialistion
	template<class Engine, class Distribution>
	class simple_generator_01
	{
	public:
		typedef typename Distribution::result_type result_type;
		simple_generator_01(Engine& e, Distribution& d)
			: urng01(e), _dist(d)
		{}
		result_type operator()()
		{	return _dist(urng01);
		 }
	private:
		class uniform_01
		{	// Simple uniform [0..1) distribution
			typedef typename Distribution::input_type result_type;
			Engine& _eng;
		public:
			uniform_01(Engine& e)
				: _eng(e)
			{}
			result_type operator()()
			{
				result_type factor = result_type(1) / result_type(_eng.max()-_eng.min() +  result_type(1)); 
              	return result_type(_eng() - _eng.min()) * factor;
			}
		};
		uniform_01 urng01;
		Distribution& _dist;
	};
	template<class Engine, class Distribution>
	class simple_generator
	{
	public:
		typedef typename Distribution::result_type result_type;
		simple_generator(Engine& e, Distribution& d)
			: _eng(e), _dist(d)
		{}
		result_type operator()()
		{	return _dist(_eng);
		 }
	private:
		Engine& _eng;
		Distribution& _dist;
	};
}//namespace

class Boost_random
{
public:
	typedef Bayesian_filter_matrix::Float Float;
	typedef boost::mt19937 URng;
	Boost_random() : dist_uniform_01(), dist_normal_01()
	{}
	Bayesian_filter_matrix::Float normal(const Float mean, const Float sigma)
	{
		boost::normal_distribution<Float> dist(mean, sigma);
		simple_generator_01<URng, boost::normal_distribution<Float> > gen(rng, dist);
		return gen();
	}
	void normal(Bayesian_filter_matrix::DenseVec& v, const Float mean, const Float sigma)
	{
		boost::normal_distribution<Float> dist(mean, sigma);
		simple_generator_01<URng, boost::normal_distribution<Float> > gen(rng, dist);
		std::generate (v.begin(), v.end(), gen);
	}
	void normal(Bayesian_filter_matrix::DenseVec& v)
	{
		simple_generator_01<URng, boost::normal_distribution<Float> > gen(rng, dist_normal_01);
		std::generate (v.begin(), v.end(), gen);
	}
	void uniform_01(Bayesian_filter_matrix::DenseVec& v)
	{
		simple_generator<URng, boost::uniform_real<Float> > gen(rng, dist_uniform_01);
		std::generate (v.begin(), v.end(), gen);
	}
#ifdef BAYES_FILTER_GAPPY
	void normal(Bayesian_filter_matrix::Vec& v, const Float mean, const Float sigma)
	{
		boost::normal_distribution<Float> dist(mean, sigma);
		simple_generator_01<URng, boost::normal_distribution<Float> > gen(rng, dist);
		for (std::size_t i = 0, iend=v.size(); i < iend; ++i)
			v[i] = gen();
	}
	void normal(Bayesian_filter_matrix::Vec& v)
	{
		simple_generator_01<URng, boost::normal_distribution<Float> > gen(rng, dist_normal_01);
		for (std::size_t i = 0, iend=v.size(); i < iend; ++i)
			v[i] = gen();
	}
	void uniform_01(Bayesian_filter_matrix::Vec& v)
	{
		simple_generator_01<URng, boost::uniform_real<Float> > gen(rng, dist_uniform_01);
		for (std::size_t i = 0, iend=v.size(); i < iend; ++i)
			v[i] = gen();
	}
#endif
	void seed()
	{
		rng.seed();
	}
private:
	URng rng;
	boost::uniform_real<Float> dist_uniform_01;
	boost::normal_distribution<Float> dist_normal_01;
};


}//namespace
