//***************************************************************************
// Copyright (C) 2007-2013 Laboratório de Sistemas e Tecnologia Subaquática *
// Departamento de Engenharia Electrotécnica e de Computadores              *
// Rua Dr. Roberto Frias, 4200-465 Porto, Portugal                          *
//***************************************************************************
// Author: Ricardo Martins                                                  *
//***************************************************************************
// $Id:: Methods.hpp 12667 2013-01-22 02:44:42Z rasm                      $:*
//***************************************************************************

#ifndef DUNE_COMPRESSION_METHODS_HPP_INCLUDED_
#define DUNE_COMPRESSION_METHODS_HPP_INCLUDED_

namespace DUNE
{
  namespace Compression
  {
    enum Methods
    {
      METHOD_ZLIB,
      METHOD_GZIP,
      METHOD_BZIP2,
      METHOD_UNKNOWN
    };
  }
}

#endif
