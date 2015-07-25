// -*-C++-*-
/*!
 * @file  creekQrCodeReaderService_impl.h
 * @brief Service implementation header of creekQrCodeReaderService.idl
 *
 */

#include "creekQrCodeReaderService.hh"


#ifndef CREEKQRCODEREADERSERVICE_IMPL_H
#define CREEKQRCODEREADERSERVICE_IMPL_H
 
/*
 * Example class implementing IDL interface OpenHRP::creekQrCodeReaderService
 */
class creekQrCodeReaderService_impl
 : public virtual POA_OpenHRP::creekQrCodeReaderService,
   public virtual PortableServer::RefCountServantBase
{
 private:
   // Make sure all instances are built on the heap by making the
   // destructor non-public
   //virtual ~creekQrCodeReaderService_impl();

 public:
   // standard constructor
   creekQrCodeReaderService_impl();
   virtual ~creekQrCodeReaderService_impl();

   // attributes and operations
   void test();

};



#endif // CREEKQRCODEREADERSERVICE_IMPL_H


