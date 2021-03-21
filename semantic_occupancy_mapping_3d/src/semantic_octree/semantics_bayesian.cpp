/**
BSD 3-Clause License
Copyright (c) 2018, Khalifa University Robotics Institute
Copyright (c) 2018, Tarek Taha tarek@tarektaha.com
Copyright (c) 2018, Reem Ashour reemashour1@gmail.com
Copyright (c) 2020, Mohamed Abdelkader mohamedashraf123@gmail.com
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include <semantics_octree/semantics_bayesian.h>
#include <algorithm>
#include <vector>

#define ALPHA 0.8
#define EPSILON 0.001

namespace octomap
{
// Struct ColorWithConfidence implementation -------------------------------------
std::ostream& operator<<(std::ostream& out, ColorWithConfidence const& c)
{
    return out << '(' << c.color << ' ' << c.confidence << ')';
}

// Struct SemanticsBayesian implementation  --------------------------------------
SemanticsBayesian SemanticsBayesian::semanticFusion(const SemanticsBayesian s1,
                                                    const SemanticsBayesian s2)
{
    // Init colors vector with s1
    std::vector<ColorWithConfidence> semantic_colors(NUM_SEMANTICS);
    for (int i = 0; i < NUM_SEMANTICS; i++)
    {
        semantic_colors[i] = s1.data[i];
    }
    std::vector<float> confidences2(NUM_SEMANTICS);
    float conf_others1 = 1., conf_others2 = 1.;  // Probability for other unknown colors
    for (int i = 0; i < NUM_SEMANTICS; i++)
    {
        conf_others1 -= s1.data[i].confidence;
        conf_others2 -= s2.data[i].confidence;
    }
    // Keep a minimum value for others where they are small to be updatable
    if (conf_others1 <= EPSILON)
        conf_others1 = EPSILON;
    if (conf_others2 <= EPSILON)
        conf_others2 = EPSILON;
    // Complete confidences2 vector with s1
    for (int i = 0; i < NUM_SEMANTICS; i++)
    {
        bool found = false;
        for (int j = 0; j < NUM_SEMANTICS; j++)
        {
            // If current color is in s2, update confidences2 with confidence of s2
            if (semantic_colors[i].color == s2.data[j].color)
            {
                confidences2[i] = s2.data[j].confidence;
                found = true;
                break;
            }
        }
        // If current color is not in s2, update confidence2 with alpha*conf_others2, also update conf_others2
        if (!found)
        {
            confidences2[i] = ALPHA * conf_others2;
            conf_others2 -= ALPHA * conf_others2;
        }
    }
    // Complete semantic_colors, confidences2 with s2
    for (int i = 0; i < NUM_SEMANTICS; i++)
    {
        bool found = false;
        for (int j = 0; j < NUM_SEMANTICS; j++)
        {
            if (s2.data[i].color == semantic_colors[j].color)
            {
                found = true;
                break;
            }
        }
        // Found new color in s2
        if (!found)
        {
            semantic_colors.push_back(ColorWithConfidence(
                s2.data[i].color, ALPHA * conf_others1));   // Add to semantic colors
            conf_others1 -= ALPHA * conf_others1;           // Update conf_others1
            confidences2.push_back(s2.data[i].confidence);  // Update confidences2
        }
    }
    // Now semantic_colors, confidences2 have the same size.
    // Perform bayesian fusion, save new confidences in semantic_colors
    float sum = conf_others1 * conf_others2;
    for (int i = 0; i < semantic_colors.size(); i++)
    {
        semantic_colors[i].confidence *= confidences2[i];  // Un-normalized confidences
        sum += semantic_colors[i].confidence;
    }
    // Normalize to a probalility distribution
    for (int i = 0; i < semantic_colors.size(); i++)
    {
        semantic_colors[i].confidence /= sum;
        // If confidence is too small, set to epsilon to prevent confidence drop to zero
        if (semantic_colors[i].confidence < EPSILON)
            semantic_colors[i].confidence = EPSILON;
    }
    // Keep top NUM_SEMANTICS colors and confidences
    std::sort(semantic_colors.begin(), semantic_colors.end());  // Asc order sorting
    SemanticsBayesian ret;
    for (int i = 0; i < NUM_SEMANTICS; i++)
    {
        ret.data[i] = semantic_colors[semantic_colors.size() - 1 - i];  // Take last elements
    }
    return ret;
}

std::ostream& operator<<(std::ostream& out, SemanticsBayesian const& s)
{
    out << '(';
    for (int i = 0; i < NUM_SEMANTICS - 1; i++)
        out << s.data[i] << ' ';
    out << s.data[NUM_SEMANTICS - 1];
    out << ')';
    return out;
}

}  //namespace octomap
