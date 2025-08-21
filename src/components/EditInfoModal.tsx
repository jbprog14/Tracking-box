"use client";

import React, { useState, useEffect } from "react";
import {
  Dialog,
  DialogContent,
  DialogHeader,
  DialogTitle,
} from "@/components/ui/dialog";
import { ScrollArea } from "@/components/ui/scroll-area";
import { Button } from "@/components/ui/button";
import { Save, X } from "lucide-react";
import { toast } from "react-hot-toast";

interface EditInfoModalProps {
  isOpen: boolean;
  onClose: () => void;
  boxId: string | null;
  currentAddress: string; // human-readable label
  currentOwner: string;
  currentDescription?: string;
  referenceCode?: string;
  
  // Sender Information
  currentSenderName?: string;
  currentSenderAddress?: string;
  
  // Recipient Information
  currentRecipientName?: string;
  currentRecipientAddress?: string;
  
  // Package Information
  currentPackDate?: string;
  currentPackWeight?: string;
  currentProductFrom?: string;
  currentPackerShipper?: string;
  currentSupplierIdTracking?: string;
  
  // Shipping Label Information
  currentRoutingCode?: string;
  currentPostalCode?: string;
  currentTrackingNumber?: string;
  currentServiceType?: string;
  
  onSave: (
    boxId: string,
    coords: string,
    owner: string,
    label: string,
    description: string,
    packageInfo: {
      packDate: string;
      packWeight: string;
      productFrom: string;
      packerShipper: string;
      supplierIdTracking: string;
      senderName: string;
      senderAddress: string;
      recipientName: string;
      recipientAddress: string;
      routingCode: string;
      postalCode: string;
      trackingNumber: string;
      serviceType: string;
    }
  ) => void;
}

export default function EditInfoModal({
  isOpen,
  onClose,
  boxId,
  currentAddress,
  currentOwner,
  currentDescription,
  referenceCode,
  currentSenderName,
  currentSenderAddress,
  currentRecipientName,
  currentRecipientAddress,
  currentPackDate,
  currentPackWeight,
  currentProductFrom,
  currentPackerShipper,
  currentSupplierIdTracking,
  currentRoutingCode,
  currentPostalCode,
  currentTrackingNumber,
  currentServiceType,
  onSave,
}: EditInfoModalProps) {
  const [address, setAddress] = useState("");
  const [description, setDescription] = useState("");
  
  // Sender Information States
  const [senderName, setSenderName] = useState("");
  const [senderAddress, setSenderAddress] = useState("");
  
  // Recipient Information States
  const [recipientName, setRecipientName] = useState("");
  const [recipientAddress, setRecipientAddress] = useState("");
  
  // Package Information States
  const [packDate, setPackDate] = useState("");
  const [packWeight, setPackWeight] = useState("");
  const [productFrom, setProductFrom] = useState("");
  const [packerShipper, setPackerShipper] = useState("");
  const [supplierIdTracking, setSupplierIdTracking] = useState("");
  
  // Shipping Label Information States
  const [routingCode, setRoutingCode] = useState("");
  const [postalCode, setPostalCode] = useState("");
  const [trackingNumber, setTrackingNumber] = useState("");
  const [serviceType, setServiceType] = useState("");
  
  const [isSaving, setIsSaving] = useState(false);
  const [geoError, setGeoError] = useState("");

  // Reset fields when modal opens
  useEffect(() => {
    if (isOpen) {
      setAddress(currentAddress || "");
      setDescription(currentDescription || "");
      
      // Initialize sender information
      setSenderName(currentSenderName || "Thanhkh Technologies Ltd");
      setSenderAddress(currentSenderAddress || "11 No.12 Dong Rd, Tsuen Wan, Hong Kong");
      
      // Initialize recipient information
      setRecipientName(currentRecipientName || "");
      setRecipientAddress(currentRecipientAddress || "");
      
      // Initialize package information
      setPackDate(currentPackDate || "");
      setPackWeight(currentPackWeight || "");
      setProductFrom(currentProductFrom || "");
      setPackerShipper(currentPackerShipper || "");
      setSupplierIdTracking(currentSupplierIdTracking || "");
      
      // Initialize shipping label information
      setRoutingCode(currentRoutingCode || "");
      setPostalCode(currentPostalCode || "");
      setTrackingNumber(currentTrackingNumber || "");
      setServiceType(currentServiceType || "D-B2C");
    }
  }, [isOpen, currentAddress, currentOwner, currentDescription,
      currentSenderName, currentSenderAddress, currentRecipientName, currentRecipientAddress,
      currentPackDate, currentPackWeight, currentProductFrom, 
      currentPackerShipper, currentSupplierIdTracking,
      currentRoutingCode, currentPostalCode, currentTrackingNumber, currentServiceType]);

  // --- Geocoding helper (tries multiple services) ---
  const geocodeAddress = async (
    query: string
  ): Promise<{ lat: string; lon: string } | null> => {
    // 1) Nominatim
    try {
      const nominatimUrl = `https://nominatim.openstreetmap.org/search?format=jsonv2&limit=1&q=${encodeURIComponent(
        query
      )}`;
      const res = await fetch(nominatimUrl, {
        headers: {
          // The browser will strip User-Agent; this is fine. We keep it for SSR environments.
          "User-Agent": "tracking-box-dashboard",
          "Accept-Language": "en",
        },
      });
      const data = await res.json();
      if (data && data.length > 0) {
        return { lat: data[0].lat, lon: data[0].lon };
      }
    } catch (err) {
      console.warn("Nominatim geocode failed", err);
    }

    // 2) Fallback – geocode.maps.co (public wrapper around OSM, CORS-enabled)
    try {
      const fallbackUrl = `https://geocode.maps.co/search?q=${encodeURIComponent(
        query
      )}`;
      const res = await fetch(fallbackUrl);
      const data = await res.json();
      if (data && data.length > 0) {
        return { lat: data[0].lat, lon: data[0].lon };
      }
    } catch (err) {
      console.warn("Fallback geocode failed", err);
    }

    return null;
  };

  const handleSave = async () => {
    // Validate required fields
    if (!boxId || !recipientName.trim() || !recipientAddress.trim()) {
      toast.error("Please fill in all required fields");
      return;
    }

    setIsSaving(true);
    setGeoError("");

    try {
      // Use recipient address for geocoding
      const addressToGeocode = recipientAddress.trim() || address.trim();
      const coords = await geocodeAddress(addressToGeocode);

      if (!coords) {
        setGeoError(
          "We couldn't locate that address. Please refine it or enter coordinates manually."
        );
        return;
      }

      const finalAddress = `${coords.lat}, ${coords.lon}`;
      
      // Package info object
      const packageInfo = {
        packDate: packDate.trim(),
        packWeight: packWeight.trim(),
        productFrom: productFrom.trim(),
        packerShipper: packerShipper.trim(),
        supplierIdTracking: supplierIdTracking.trim(),
        senderName: senderName.trim(),
        senderAddress: senderAddress.trim(),
        recipientName: recipientName.trim(),
        recipientAddress: recipientAddress.trim(),
        routingCode: routingCode.trim(),
        postalCode: postalCode.trim(),
        trackingNumber: trackingNumber.trim(),
        serviceType: serviceType.trim()
      };
      
      onSave(
        boxId,
        finalAddress,
        recipientName.trim(), // Use recipient name as owner
        recipientAddress.trim() || address.trim(),
        description.trim(),
        packageInfo
      );
      onClose();
    } catch (error) {
      console.error("Geocoding error:", error);
      setGeoError("Unexpected error while locating address.");
    } finally {
      setIsSaving(false);
    }
  };

  const handleCancel = () => {
    setAddress(currentAddress || "");
    setDescription(currentDescription || "");
    setSenderName(currentSenderName || "Thanhkh Technologies Ltd");
    setSenderAddress(currentSenderAddress || "11 No.12 Dong Rd, Tsuen Wan, Hong Kong");
    setRecipientName(currentRecipientName || "");
    setRecipientAddress(currentRecipientAddress || "");
    setPackDate(currentPackDate || "");
    setPackWeight(currentPackWeight || "");
    setProductFrom(currentProductFrom || "");
    setPackerShipper(currentPackerShipper || "");
    setSupplierIdTracking(currentSupplierIdTracking || "");
    setRoutingCode(currentRoutingCode || "");
    setPostalCode(currentPostalCode || "");
    setTrackingNumber(currentTrackingNumber || "");
    setServiceType(currentServiceType || "D-B2C");
    onClose();
  };

  return (
    <Dialog open={isOpen} onOpenChange={onClose}>
      <DialogContent className="max-w-6xl w-[95vw] max-h-[95vh] h-[90vh] p-0 flex flex-col">
        <DialogHeader className="px-8 pt-6 pb-4 border-b bg-gray-50">
          <DialogTitle className="text-2xl font-bold text-gray-900">
            Shipping Label & Package Information
          </DialogTitle>
          {boxId && (
            <div className="flex items-center gap-4 mt-2">
              <div className="inline-flex items-center px-4 py-2 rounded-lg text-base font-semibold bg-blue-100 text-blue-800 border border-blue-300">
                Box ID: {boxId.toUpperCase()}
              </div>
              {referenceCode && (
                <div className="text-sm text-gray-600 font-mono bg-gray-100 px-3 py-1 rounded">
                  Reference: {referenceCode}
                </div>
              )}
            </div>
          )}
        </DialogHeader>

        <ScrollArea className="flex-1 overflow-auto">
          <div className="px-10 py-8">
            {/* Main Grid Layout */}
            <div className="grid grid-cols-2 gap-10">
              
              {/* Left Column */}
              <div className="space-y-7">
                
                {/* Sender Information Section */}
                <div className="bg-white p-7 rounded-lg border border-gray-200 shadow-sm">
                  <h3 className="text-xl font-bold text-gray-900 mb-6 pb-3 border-b border-gray-100">
                    Sender Information
                  </h3>
                  
                  <div className="space-y-5">
                    <div>
                      <label className="block text-sm font-semibold text-gray-700 mb-2.5 uppercase tracking-wider">
                        Sender Name <span className="text-red-500">*</span>
                      </label>
                      <input
                        type="text"
                        value={senderName}
                        onChange={(e) => setSenderName(e.target.value)}
                        placeholder="Company or sender name"
                        className="w-full px-4 py-3.5 border-2 border-gray-600 rounded-lg text-base focus:border-blue-500 focus:ring-2 focus:ring-blue-100 focus:outline-none transition-all"
                        maxLength={100}
                      />
                      <div className="text-xs text-gray-400 mt-1">{senderName.length}/100</div>
                    </div>
                    
                    <div>
                      <label className="block text-sm font-semibold text-gray-700 mb-2.5 uppercase tracking-wider">
                        Sender Address <span className="text-red-500">*</span>
                      </label>
                      <textarea
                        value={senderAddress}
                        onChange={(e) => setSenderAddress(e.target.value)}
                        placeholder="Full sender address"
                        className="w-full px-4 py-3.5 border-2 border-gray-600 rounded-lg resize-none h-28 text-base focus:border-blue-500 focus:ring-2 focus:ring-blue-100 focus:outline-none transition-all"
                        maxLength={200}
                      />
                      <div className="text-xs text-gray-400 mt-1">{senderAddress.length}/200</div>
                    </div>
                  </div>
                </div>

                {/* Recipient Information Section */}
                <div className="bg-white p-7 rounded-lg border border-gray-200 shadow-sm">
                  <h3 className="text-xl font-bold text-gray-900 mb-6 pb-3 border-b border-gray-100">
                    Recipient Information (Ship To)
                  </h3>
                  
                  <div className="space-y-5">
                    <div>
                      <label className="block text-sm font-semibold text-gray-700 mb-2.5 uppercase tracking-wider">
                        Recipient Name <span className="text-red-500">*</span>
                      </label>
                      <input
                        type="text"
                        value={recipientName}
                        onChange={(e) => setRecipientName(e.target.value)}
                        placeholder="Recipient's name"
                        className="w-full px-4 py-3.5 border-2 border-gray-600 rounded-lg text-base focus:border-blue-500 focus:ring-2 focus:ring-blue-100 focus:outline-none transition-all"
                        maxLength={100}
                        required
                      />
                      <div className="text-xs text-gray-400 mt-1">{recipientName.length}/100</div>
                    </div>
                    
                    <div>
                      <label className="block text-sm font-semibold text-gray-700 mb-2.5 uppercase tracking-wider">
                        Recipient Address <span className="text-red-500">*</span>
                      </label>
                      <textarea
                        value={recipientAddress}
                        onChange={(e) => setRecipientAddress(e.target.value)}
                        placeholder="Full delivery address"
                        className="w-full px-4 py-3.5 border-2 border-gray-600 rounded-lg resize-none h-28 text-base focus:border-blue-500 focus:ring-2 focus:ring-blue-100 focus:outline-none transition-all"
                        maxLength={200}
                        required
                      />
                      <div className="text-xs text-gray-400 mt-1">{recipientAddress.length}/200</div>
                    </div>
                  </div>
                  
                  {geoError && (
                    <div className="bg-red-50 border border-red-200 rounded-lg p-3 mt-4">
                      <p className="text-red-700 text-sm">{geoError}</p>
                    </div>
                  )}
                </div>

              </div>

              {/* Right Column */}
              <div className="space-y-7">
                
                {/* Shipping Label Details */}
                <div className="bg-white p-7 rounded-lg border border-gray-200 shadow-sm">
                  <h3 className="text-xl font-bold text-gray-900 mb-6 pb-3 border-b border-gray-100">
                    Shipping Label Details
                  </h3>
                  
                  <div className="grid grid-cols-2 gap-5">
                    <div>
                      <label className="block text-sm font-semibold text-gray-700 mb-2.5 uppercase tracking-wider">
                        Tracking Number
                      </label>
                      <input
                        type="text"
                        value={trackingNumber}
                        onChange={(e) => setTrackingNumber(e.target.value)}
                        placeholder="ZH-0522"
                        className="w-full px-4 py-3.5 border-2 border-gray-600 rounded-lg text-base font-mono focus:border-blue-500 focus:ring-2 focus:ring-blue-100 focus:outline-none transition-all"
                        maxLength={50}
                      />
                    </div>
                    
                    <div>
                      <label className="block text-sm font-semibold text-gray-700 mb-2.5 uppercase tracking-wider">
                        Service Type
                      </label>
                      <select
                        value={serviceType}
                        onChange={(e) => setServiceType(e.target.value)}
                        className="w-full px-4 py-3.5 border-2 border-gray-600 rounded-lg text-base focus:border-blue-500 focus:ring-2 focus:ring-blue-100 focus:outline-none transition-all"
                      >
                        <option value="D-B2C">D-B2C</option>
                        <option value="D-B2B">D-B2B</option>
                        <option value="EXPRESS">EXPRESS</option>
                        <option value="STANDARD">STANDARD</option>
                      </select>
                    </div>
                    
                    <div>
                      <label className="block text-sm font-semibold text-gray-700 mb-2.5 uppercase tracking-wider">
                        Routing Code
                      </label>
                      <input
                        type="text"
                        value={routingCode}
                        onChange={(e) => setRoutingCode(e.target.value)}
                        placeholder="0522"
                        className="w-full px-4 py-3.5 border-2 border-gray-600 rounded-lg text-base font-mono focus:border-blue-500 focus:ring-2 focus:ring-blue-100 focus:outline-none transition-all"
                        maxLength={20}
                      />
                    </div>
                    
                    <div>
                      <label className="block text-sm font-semibold text-gray-700 mb-2.5 uppercase tracking-wider">
                        Postal Code
                      </label>
                      <input
                        type="text"
                        value={postalCode}
                        onChange={(e) => setPostalCode(e.target.value)}
                        placeholder="8992 0843 67"
                        className="w-full px-4 py-3.5 border-2 border-gray-600 rounded-lg text-base font-mono focus:border-blue-500 focus:ring-2 focus:ring-blue-100 focus:outline-none transition-all"
                        maxLength={20}
                      />
                    </div>
                  </div>
                </div>

                {/* Package Information */}
                <div className="bg-white p-7 rounded-lg border border-gray-200 shadow-sm">
                  <h3 className="text-xl font-bold text-gray-900 mb-6 pb-3 border-b border-gray-100">
                    Package Information
                  </h3>
                  
                  <div className="grid grid-cols-2 gap-5">
                    <div>
                      <label className="block text-sm font-semibold text-gray-700 mb-2.5 uppercase tracking-wider">
                        Package Weight
                      </label>
                      <input
                        type="text"
                        value={packWeight}
                        onChange={(e) => setPackWeight(e.target.value)}
                        placeholder="10.00KG"
                        className="w-full px-4 py-3.5 border-2 border-gray-600 rounded-lg text-base focus:border-blue-500 focus:ring-2 focus:ring-blue-100 focus:outline-none transition-all"
                        maxLength={50}
                      />
                    </div>
                    
                    <div>
                      <label className="block text-sm font-semibold text-gray-700 mb-2.5 uppercase tracking-wider">
                        Pack Date
                      </label>
                      <input
                        type="date"
                        value={packDate}
                        onChange={(e) => setPackDate(e.target.value)}
                        className="w-full px-4 py-3.5 border-2 border-gray-600 rounded-lg text-base focus:border-blue-500 focus:ring-2 focus:ring-blue-100 focus:outline-none transition-all"
                      />
                    </div>
                    
                    <div className="col-span-2">
                      <label className="block text-sm font-semibold text-gray-700 mb-2.5 uppercase tracking-wider">
                        Product Origin
                      </label>
                      <input
                        type="text"
                        value={productFrom}
                        onChange={(e) => setProductFrom(e.target.value)}
                        placeholder="Origin or source location"
                        className="w-full px-4 py-3.5 border-2 border-gray-600 rounded-lg text-base focus:border-blue-500 focus:ring-2 focus:ring-blue-100 focus:outline-none transition-all"
                        maxLength={100}
                      />
                      <div className="text-xs text-gray-400 mt-1">{productFrom.length}/100</div>
                    </div>
                    
                    <div className="col-span-2">
                      <label className="block text-sm font-semibold text-gray-700 mb-2.5 uppercase tracking-wider">
                        Packer / Shipper
                      </label>
                      <input
                        type="text"
                        value={packerShipper}
                        onChange={(e) => setPackerShipper(e.target.value)}
                        placeholder="Name of packer or shipping company"
                        className="w-full px-4 py-3.5 border-2 border-gray-600 rounded-lg text-base focus:border-blue-500 focus:ring-2 focus:ring-blue-100 focus:outline-none transition-all"
                        maxLength={100}
                      />
                      <div className="text-xs text-gray-400 mt-1">{packerShipper.length}/100</div>
                    </div>
                    
                    <div className="col-span-2">
                      <label className="block text-sm font-semibold text-gray-700 mb-2.5 uppercase tracking-wider">
                        Supplier ID / Additional Tracking
                      </label>
                      <input
                        type="text"
                        value={supplierIdTracking}
                        onChange={(e) => setSupplierIdTracking(e.target.value)}
                        placeholder="Supplier ID or additional tracking number"
                        className="w-full px-4 py-3.5 border-2 border-gray-600 rounded-lg text-base focus:border-blue-500 focus:ring-2 focus:ring-blue-100 focus:outline-none transition-all"
                        maxLength={100}
                      />
                      <div className="text-xs text-gray-400 mt-1">{supplierIdTracking.length}/100</div>
                    </div>
                    
                    <div className="col-span-2">
                      <label className="block text-sm font-semibold text-gray-700 mb-2.5 uppercase tracking-wider">
                        Item Description
                      </label>
                      <textarea
                        value={description}
                        onChange={(e) => setDescription(e.target.value)}
                        placeholder="Package contents description..."
                        className="w-full px-4 py-3.5 border-2 border-gray-600 rounded-lg resize-none h-24 text-base focus:border-blue-500 focus:ring-2 focus:ring-blue-100 focus:outline-none transition-all"
                        maxLength={500}
                      />
                      <div className="text-xs text-gray-400 mt-1">{description.length}/500</div>
                    </div>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </ScrollArea>

        {/* Action Buttons - Fixed at bottom */}
        <div className="px-8 py-6 border-t bg-gray-50">
          <div className="flex justify-center gap-4">
            <Button
              onClick={handleCancel}
              variant="outline"
              className="w-40 flex items-center justify-center gap-2 border-2 border-gray-400 rounded-lg hover:bg-gray-100 hover:border-gray-500 text-gray-700 py-3 text-base font-semibold"
            >
              <X className="h-5 w-5" />
              Cancel
            </Button>
            <Button
              onClick={handleSave}
              disabled={!recipientName.trim() || !recipientAddress.trim() || isSaving}
              className="w-40 flex items-center justify-center gap-2 bg-blue-600 hover:bg-blue-700 text-white border-2 border-blue-700 rounded-lg disabled:opacity-50 disabled:cursor-not-allowed disabled:bg-gray-400 disabled:border-gray-500 py-3 text-base font-semibold"
            >
              {isSaving ? (
                "Saving..."
              ) : (
                <>
                  <Save className="h-5 w-5" />
                  Save Changes
                </>
              )}
            </Button>
          </div>

          {/* Warning Text */}
          <p className="text-sm text-gray-600 text-center bg-yellow-50 p-3 rounded-lg border border-yellow-200 mt-4">
            Note: Changes will be applied immediately and synced to the database. Required fields are marked with <span className="text-red-500">*</span>
          </p>
        </div>
      </DialogContent>
    </Dialog>
  );
}