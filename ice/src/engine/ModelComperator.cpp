/*
 * ModelComperator.cpp
 *
 *  Created on: Jun 5, 2014
 *      Author: sni
 */

#define DEBUG_NO

#include <ice/coordination/ModelComperator.h>

namespace ice
{

ModelComperator::ModelComperator()
{
  // TODO Auto-generated constructor stub

}

ModelComperator::~ModelComperator()
{
  // TODO Auto-generated destructor stub
}

std::shared_ptr<std::vector<std::shared_ptr<IntersectionInformationModel>>>ModelComperator::findModelMatches(std::shared_ptr<InformationModel> model1,
    std::shared_ptr<InformationModel> model2)
{
  std::vector<std::shared_ptr<StreamDescription>> sharedStreams;
  std::vector<std::shared_ptr<StreamTemplateDescription>> sharedStreamTemplates;
  std::vector<std::shared_ptr<NodeDescription>> sharedNodes;

  // find equal streams
  for (auto u1 : *model1->getStreams())
  {
    for (auto u2 : *model2->getStreams())
    {
      if (u1->equals(u2.get()))
      {
        sharedStreams.push_back(u1);
#ifdef  DEBUG
        std::cout << "MC: Equal stream " << u1->getUuid() << " " << u1->isShared() << std::endl;
#endif
        break;
      }
    }
  }

  // find equal stream templates
  for (auto u1 : *model1->getStreamTemplates())
  {
    for (auto u2 : *model2->getStreamTemplates())
    {
      if (u1->equals(u2.get()))
      {
        sharedStreamTemplates.push_back(u1);
#ifdef  DEBUG
        std::cout << "MC: Equal stream template " << u1->getUuid() << std::endl;
#endif
        break;
      }
    }
  }

  // find equal nodes
  for (auto u1 : *model1->getNodeDescriptions())
  {
    for (auto u2 : *model2->getNodeDescriptions())
    {
      if (false == u1->equals(u2.get()))
      {
        continue;
      }

      // check inputs
      int count = 0;
      const boost::uuids::uuid* inputs = u2->getInputUuids(&count);
      int foundEquals = 0;

      for (int i = 0; i < count; ++i)
      {
        for (auto stream : sharedStreams)
        {
          if (inputs[i] == stream->getUuid())
          {
            ++foundEquals;
            break;
          }
        }
      }

      if (count != foundEquals)
      {
        continue;
      }

      // check input templates
      int inputCount = count;
      count = 0;
      const boost::uuids::uuid* templates = u2->getInputTemplateUuids(&count);
      foundEquals = 0;

      for (int i = 0; i < count; ++i)
      {
        for (auto streamTemplate : sharedStreamTemplates)
        {
          if (templates[i] == streamTemplate->getUuid())
          {
            ++foundEquals;
            break;
          }
        }
      }

      if (count != foundEquals || count + inputCount == 0)
      {
        continue;
      }

      // check outputs
      count = 0;
      const boost::uuids::uuid* outputs = u2->getOutputUuids(&count);
      foundEquals = 0;

      for (int i = 0; i < count; ++i)
      {
        for (auto stream : sharedStreams)
        {
          if (outputs[i] == stream->getUuid())
          {
            ++foundEquals;
            break;
          }
        }
      }

      if (count != foundEquals || count == 0)
      {
        continue;
      }

#ifdef  DEBUG
      std::cout << "MC: Equal node " << u1->getClassName() << std::endl;
#endif
      sharedNodes.push_back(u1);
    }
  }

  auto matches = std::make_shared<std::vector<std::shared_ptr<IntersectionInformationModel>>>();

// No matching
  if (sharedStreams.size() == 0 || sharedStreamTemplates.size() == 0 || sharedNodes.size() == 0)
  return matches;

// Create connection matrix
  int nodeCount = sharedNodes.size();
  short cm[nodeCount * nodeCount];
  std::shared_ptr<NodeDescription> nodeDesc;

  for (int i=0; i < nodeCount * nodeCount; ++i)
  {
    cm[i] = 0;
  }

  for (int i=0; i < nodeCount; ++i)
  {
    nodeDesc = sharedNodes[i];
    int inputSize, inputTemplateSize, outputSize;
    const boost::uuids::uuid* inputs = nodeDesc->getInputUuids(&inputSize);
    const boost::uuids::uuid* inputTemplates = nodeDesc->getInputTemplateUuids(&inputTemplateSize);
    const boost::uuids::uuid* outputs = nodeDesc->getOutputUuids(&outputSize);
    bool found = false;

    // check if node is an input node
    for (int j = 0; j < inputSize; ++j)
    {
      for (int k = 0; k < inputTemplateSize; ++k)
      {
        if (inputs[j] == inputTemplates[k])
        {
          found = true;
          cm[i * nodeCount + i] = 1;
          break;
        }
      }

      if (found)
      break;
    }

    // Check if node is an output node
    found = false;
    for (int j = 0; j < outputSize; ++j)
    {
      for (int k = 0; k < sharedStreams.size(); ++k)
      {
        if (outputs[j] == sharedStreams[k]->getUuid())
        {
          if (sharedStreams[k]->isShared())
          {
            cm[i * nodeCount + i] += 2;
            found = true;
          }
          break;
        }
      }
      if (found)
      break;
    }

    // Check outputs
    std::vector<int> toVisit;
    short visitNodes[nodeCount];

    for (int i = 0; i < nodeCount; ++i)
    {
      visitNodes[i] = 0;
    }

    toVisit.push_back(i);

    while (toVisit.size() > 0)
    {
      int nodeId = toVisit.back();
      toVisit.pop_back();

      if (visitNodes[nodeId])
      continue;

      visitNodes[nodeId] = 1;

      for (int j = 0; j < outputSize; ++j)
      {
        found = false;

        for (int k = 0; k < nodeCount; ++k)
        {
          if (k == i)
          continue;

          if (sharedNodes[k]->existsInInputUuids(outputs[j]))
          {
            toVisit.push_back(k);
            cm[i * nodeCount + k] = 1;
          }
        }
      }
    }
  }

#ifdef DEBUG
  for (int i=0; i < nodeCount * nodeCount; ++i)
  {
    if (i != 0 && i % nodeCount == 0)
    {
      std::cout << std::endl;
    }
    std::cout << cm[i] << " ";
  }
  std::cout << std::endl;
#endif

// Find matchings

  int currentRow;
  short visitNodes[nodeCount];
  short usedNodes[nodeCount];
  short inputNodes[nodeCount];
  short outputNodes[nodeCount];
  bool valid;
  std::vector<int> toVisit;

  for (int i = 0; i < nodeCount; ++i)
  {
    visitNodes[i] = 0;
  }

  for (int i = 0; i < nodeCount; ++i)
  {
#ifdef DEBUG
    std::cout << "-----------------" << std::endl;
    std::cout << "MC: Start " << i << std::endl;
#endif

    currentRow = i * nodeCount;
    if (false == (cm[currentRow + i] & 1))
    {
#ifdef DEBUG
      std::cout << "MC: No input node " << std::endl;
#endif
      continue;
    }

    valid = false;
    for (int j = 0; j < nodeCount; ++j)
    {
      usedNodes[j] = 0;
      inputNodes[j] = 0;
      outputNodes[j] = 0;
    }

    inputNodes[i] = 1;

    // usedNodes[i] = 1;
    if (cm[currentRow + i] == 3)
    {
      valid = true;
      outputNodes[i] = 1;
    }

    toVisit.push_back(i);

    while (false == toVisit.empty())
    {
      int nodeId = toVisit.back();
      toVisit.pop_back();

      if (usedNodes[nodeId])
      continue;

#ifdef DEBUG
      std::cout << "MC: Check " << nodeId << std::endl;
#endif

      usedNodes[nodeId] = 1;

      if (cm[nodeId * nodeCount + nodeId] == 3)
      {
        valid = true;
      }

      if (cm[nodeId * nodeCount + nodeId] >= 2)
      {
#ifdef DEBUG
        std::cout << "MC: Add to output " << nodeId << std::endl;
#endif
        outputNodes[nodeId] = 1;
      }
      else if (cm[nodeId * nodeCount + nodeId] & 1)
      {
#ifdef DEBUG
        std::cout << "MC: Add to input " << nodeId << std::endl;
#endif
        inputNodes[nodeId] = 1;
      }

      for (int j = 0; j < nodeCount; ++j)
      {
        if (usedNodes[j])
        continue;

#ifdef DEBUG
        std::cout << "MC: Check from " << nodeId << " to " << j << std::endl;
#endif

        if (j != nodeId)
        {
          if (cm[nodeId * nodeCount + j])
          {
#ifdef DEBUG
            std::cout << "MC: Add to list from " << nodeId << " to " << j << std::endl;
#endif
            toVisit.push_back(j);
          }
        }
      }

      for (int j = 0; j < nodeCount; ++j)
      {
        if (j != nodeId)
        {
          if (cm[j * nodeCount + nodeId])
          {
#ifdef DEBUG
            std::cout << "MC: Add to list from " << nodeId << " to " << j << std::endl;
#endif
            toVisit.push_back(j);
          }
        }
      }
    }

#ifdef DEBUG
    std::cout << "MC: Valid " << valid << std::endl;
#endif

    if (false == valid)
    continue;

    bool found = false;
    for (int j = 0; j < nodeCount; ++j)
    {
      if (usedNodes[j] == 1 && visitNodes[j] == 1)
      {
        found = true;
        break;
      }
    }

    if (found)
    {
#ifdef DEBUG
      std::cout << "MC: Duplicated use of nodes" << std::endl;
#endif
      continue;
    }

    auto match = std::make_shared<IntersectionInformationModel>();
    int nCounter = 0;

    for (int j = 0; j < nodeCount; ++j)
    {
      nodeDesc = sharedNodes[j];

      if (usedNodes[j])
      {
        visitNodes[j] = 1;
        match->getNodeDescriptions()->push_back(nodeDesc);
        nCounter++;
      }

      if (inputNodes[j])
      {
#ifdef DEBUG
        std::cout << "MC: Match add inputs from node " << j << std::endl;
#endif
        int inputSize, inputTemplateSize;
        const boost::uuids::uuid* inputs = nodeDesc->getInputUuids(&inputSize);
        const boost::uuids::uuid* inputTemplates = nodeDesc->getInputTemplateUuids(&inputTemplateSize);

        for (int k = 0; k < inputSize; ++k)
        {
          for (int l = 0; l < inputTemplateSize; ++l)
          {
            if (inputs[k] == inputTemplates[l])
            {
              match->addToInput(model1->getStreamTemplateByUuid(inputs[k]));
              break;
            }
          }
        }
      }

      if (outputNodes[j])
      {
#ifdef DEBUG
        std::cout << "MC: Match add output from node " << j << std::endl;
#endif
        int outputSize;
        const boost::uuids::uuid* outputs = nodeDesc->getOutputUuids(&outputSize);

        for (int k = 0; k < outputSize; ++k)
        {
          for (int l = 0; l < sharedStreams.size(); ++l)
          {
            if (outputs[k] == sharedStreams[l]->getUuid())
            {
              if (sharedStreams[l]->isShared())
              {
                match->addToOutput(model1->getStreamByUuid(outputs[k]));
              }
              break;
            }
          }
        }
      }
    }

    short* shrinkedConnectionMatrix = new short[nCounter * nCounter];
    int rowCount = 0;
    int coloumCount = 0;

    for (int j = 0; j < nodeCount; ++j)
    {
      if (usedNodes[j] == 0)
      continue;

      coloumCount = 0;

      for (int k = 0; k < nodeCount; ++k)
      {
        if (usedNodes[k] == 0)
        continue;

#ifdef DEBUG
        std::cout << "MC: Add to cm from  " << j << " " << k << " to " << rowCount << " " << coloumCount << " value " << cm[j * nodeCount + k] << std::endl;
#endif

        shrinkedConnectionMatrix[rowCount * nCounter + coloumCount] = cm[j * nodeCount + k];

        coloumCount++;
      }

      ++rowCount;
    }

    match->setConnectionMatrix(shrinkedConnectionMatrix);

    matches->push_back(match);
  }

  return matches;
}

bool ModelComperator::findOfferesAndRequests(std::shared_ptr<InformationModel> model1,
                                             std::shared_ptr<InformationModel> model2,
                                             std::shared_ptr<std::vector<std::shared_ptr<StreamDescription>>>offers,
                                             std::shared_ptr<std::vector<std::shared_ptr<StreamTemplateDescription>>>requests)
{
  const std::vector<std::shared_ptr<StreamDescription>>* model1Streams = model1->getStreams();
  const std::vector<std::shared_ptr<StreamTemplateDescription>>* model1StreamTemplates = model1->getStreamTemplates();
  const std::vector<std::shared_ptr<StreamDescription>>* model2Streams = model2->getStreams();
  const std::vector<std::shared_ptr<StreamTemplateDescription>>* model2StreamTemplates = model2->getStreamTemplates();

  bool foundMatches = false;

  // find offeres from model 1 to model 2
  for (auto u1 : *model1Streams)
  {
    for (auto u2 : *model2StreamTemplates)
    {
      if (u1->isShared() && u1->equals(u2.get()))
      {
        foundMatches = true;
        offers->push_back(u1);
      }
    }
  }

  // find requests from model 1 to model 2
  for (auto u1 : *model2Streams)
  {
    for (auto u2 : *model1StreamTemplates)
    {
      if (u1->isShared() && u1->equals(u2.get()))
      {
        foundMatches = true;
        requests->push_back(u2);
      }
    }
  }

  return foundMatches;
}

/*
 bool ModelComperator::findMatchings(const std::vector<boost::uuids::uuid>* uuids1,
 const std::vector<boost::uuids::uuid>* uuids2,
 std::shared_ptr<std::vector<boost::uuids::uuid> > matches)
 {
 bool foundMatches = false;

 for (auto u1 : *uuids1)
 {
 for (auto u2 : *uuids2)
 {
 if (u1 == u2)
 {
 foundMatches = true;
 matches->push_back(u2);
 }
 }
 }

 return foundMatches;
 }*/

} /* namespace ice */
