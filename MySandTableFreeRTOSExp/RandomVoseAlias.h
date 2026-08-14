/******************************************************************************
 * File: RandomVoseAlias.h
 * Author: Keith Schwarz (htiek@cs.stanford.edu), converted to C++ by Joe Corbett.
 *
 * An implementation of the m_Alias method implemented using Vose's algorithm.
 * The m_Alias method allows for efficient sampling of random values from a
 * discrete probability distribution (i.e. rolling a loaded die) in O(1) time
 * each after O(n) preprocessing time.
 *
 * For a complete writeup on the alias method, including the intuition and
 * important proofs, please see the article "Darts, Dice, and Coins: Sampling
 * from a Discrete Distribution" at
 *
 *                 http://www.keithschwarz.com/darts-dice-coins/
 *
 * History:
 * - 28-JUL-2025 Joe Corbett (JMC)
 *   - Converted to C++.
 * - 04-AUG-2026 JMC
 *   - Added m_pNextRandom function pointer so that the class could be used
 *     in various applications.
 *******************************************************************************/
#include <vector>
#include <deque>

typedef int32_t (*NextRng_t)(int32_t, int32_t);


class RandomVoseAlias
{
public:
    /****************************************************************************
     * Constructs a new new instance .  Requires a call to Init() before the
     * instance can be used.
     ***************************************************************************/
    RandomVoseAlias(NextRng_t pRng) : m_pNextRandom(pRng), m_InitDone(false) {}


    /****************************************************************************
     * Constructs a new RandomVoseAlias to sample from a discrete distribution
     * and hand back outcomes based on the probability distribution.
     *
     * @param probabilities The list of probabilities.
     ***************************************************************************/
    RandomVoseAlias(const std::vector<float_t>& probabilities) : m_InitDone(false)
    {
        Init(probabilities);
    }


    /****************************************************************************
     * Initializes an existing RandomVoseAlias to sample from a discrete
     * distribution and hand back outcomes based on the probability distribution.
     *
     * @param probabilities The list of probabilities.
     ***************************************************************************/
    bool Init(const std::vector<float_t>& probabilities);


    /****************************************************************************
     * Samples a value from the underlying distribution.
     *
     * @return A random value sampled from the underlying distribution.
     ***************************************************************************/
    int Next();


private:
    /* The m_Probability and m_Alias tables. */
    NextRng_t            m_pNextRandom;      // Pointer to random generator function.
    std::vector<int>     m_Alias;            // Alias vector.
    std::vector<float_t> m_Probability;      // Probability vector.
    bool                 m_InitDone;         // False until initialization is complete.
};
