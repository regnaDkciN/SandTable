/******************************************************************************
 * File: RandomVoseAlias.cpp
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
 * Converted to C++ by Joe Corrbett 28-JUL-2025.
 *******************************************************************************/
#include <Arduino.h>            // For random().
#include "RandomVoseAlias.h"    // For RandomVoseAlias class.


/****************************************************************************
 * Initializes an existing RandomVoseAlias to sample from a discrete
 * distribution and hand back outcomes based on the probability distribution.
 *
 * @param probabilities The list of probabilities.
 ***************************************************************************/
bool RandomVoseAlias::Init(const std::vector<float_t>& probabilities)
{
    /* Begin by doing basic structural checks on the inputs. */
    if (probabilities.empty())
    {
        return false;
    }

    /* Allocate space for the m_Probability and m_Alias tables. */
    this->m_Probability.resize(probabilities.size());
    this->m_Alias.resize(probabilities.size());

    /* Make a copy of the probabilities table so that we can modify it as needed., */
    std::vector<float_t> probabilitiesCopy(probabilities);

    /* Compute the average m_Probability and cache it for later use. */
    const float_t average = 1.0 / probabilities.size();

    /* Create two deques to act as worklists as we populate the tables. */
    std::deque<int> small;
    std::deque<int> large;

    /* Populate the deques with the input probabilities. */
    for (size_t i = 0; i < probabilitiesCopy.size(); ++i)
    {
        /* If the m_Probability is below the average m_Probability, then we add
         * it to the small list; otherwise we add it to the large list.
         */
        if (probabilitiesCopy[i] >= average)
        {
            large.push_back(i);
        }
        else
        {
            small.push_back(i);
        }
    }

    /* Pair small and large elements. */
    while (!small.empty() && !large.empty())
    {
        /* Get the index of the small and the large probabilities. */
        int less = small.back();
        small.pop_back();
        int more = large.back();
        large.pop_back();

        /* Scale the probabilities. */
        m_Probability[less] = probabilitiesCopy[less] * probabilitiesCopy.size();
        m_Alias[less] = more;

        /* Decrease the m_Probability of the larger one by the appropriate amount. */
        probabilitiesCopy[more] = (probabilitiesCopy[more] + probabilitiesCopy[less]) - average;

        /* Add the new m_Probability to the appropriate list. */
        if (probabilitiesCopy[more] >= average)
        {
            large.push_back(more);
        }
        else
        {
            small.push_back(more);
        }
    }

    /* Set remaining probabilities to 1/n. */
    while (!small.empty())
    {
        m_Probability[small.back()] = 1.0;
        small.pop_back();
    }
    while (!large.empty())
    {
        m_Probability[large.back()] = 1.0;
        large.pop_back();
    }

    m_InitDone = true;
    return true;
} // End constructor.


/********************************************************************************
* Samples a value from the underlying distribution.
*
* @return A random value sampled from the underlying distribution.
********************************************************************************/
int RandomVoseAlias::Next()
{
    const int32_t PROBABILITY_RESOLUTION = 100000;

    // Make sure we don't try to generate a random value if we haven't been
    // initialized yet.
    if (!m_InitDone)
    {
        return 0;
    }

    /* Generate a fair die roll to determine which column to inspect. */
    int column = random(0, m_Probability.size());

    /* Generate a biased coin toss to determine which option to pick. */
    float_t rnd = (float_t)random(0, PROBABILITY_RESOLUTION) / (float_t)PROBABILITY_RESOLUTION;
    bool coinToss = rnd < m_Probability[column];

    /* Based on the outcome, return either the column or its m_Alias. */
    return coinToss ? column : m_Alias[column];
} // End Next().

