/*
 * Model.h
 *
 *  Created on: Jun 5, 2014
 *      Author: sni
 */

#ifndef MODEL_H_
#define MODEL_H_

#include <memory>
#include <vector>

#include "boost/uuid/uuid.hpp"

// Forward declaration
namespace ice
{
class NodeDescription;
class StreamDescription;
class StreamTemplateDescription;
} /* namespace ice */

namespace ice
{

//* InformationModel
/**
 * This class is an abstract description of the information structure.
 *
 */
class InformationModel
{
public:
  /*!
   * \brief Default constructor.
   *
   * Default constructor.
   */
  InformationModel();

  /*!
   * \brief Default destructor
   *
   * Default destructor
   */
  virtual ~InformationModel();

  /*!
    * \brief Returns a reference to the node description vector.
    *
    * Returns a reference to the node description vector.
    */
  std::vector<std::shared_ptr<NodeDescription>>* getNodeDescriptions();

  /*!
    * \brief Returns a reference to the stream vector.
    *
    * Returns a reference to the stream vector.
    */
  std::vector<std::shared_ptr<StreamDescription>>* getStreams();

  /*!
   * \brief Returns a stream description for a given uuid.
   *
   * Returns a stream description for a given uuid.
   *
   * \param uuid The uuid of the stream.
   */
  std::shared_ptr<StreamDescription> getStreamByUuid(boost::uuids::uuid uuid) const;

  /*!
   * \brief Returns a reference to the streamTemplate vector.
   *
   * Returns a reference to the streamTemplate vector.
   */
  std::vector<std::shared_ptr<StreamTemplateDescription>>* getStreamTemplates();

  /*!
   * \brief Returns a stream template description for a given uuid.
   *
   * Returns a stream template description for a given uuid.
   *
   * \param uuid The uuid of the stream template.
   */
  std::shared_ptr<StreamTemplateDescription> getStreamTemplateByUuid(boost::uuids::uuid uuid) const;

protected:
  std::vector<std::shared_ptr<NodeDescription>> nodeDescriptions; /**< Vector of node descriptions */
  std::vector<std::shared_ptr<StreamDescription>> streams; /**< List of information uuids where at least an information stream exists within the engine */
  std::vector<std::shared_ptr<StreamTemplateDescription>> streamTemplates; /**< List of information uuids where at least an information stream template exists */
};

} /* namespace ice */

#endif /* MODEL_H_ */
