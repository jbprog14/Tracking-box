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
import { MapPin, User, Save, X, FileText, Package, Calendar, Weight, Truck, Hash } from "lucide-react";
import { toast } from "react-hot-toast";

interface EditInfoModalProps {
  isOpen: boolean;
  onClose: () => void;
  boxId: string | null;
  currentAddress: string; // human-readable label
  currentOwner: string;
  currentDescription?: string;
  referenceCode?: string;
  
  // Package Information
  currentPackDate?: string;
  currentPackWeight?: string;
  currentProductFrom?: string;
  currentPackerShipper?: string;
  currentSupplierIdTracking?: string;
  
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
  currentPackDate,
  currentPackWeight,
  currentProductFrom,
  currentPackerShipper,
  currentSupplierIdTracking,
  onSave,
}: EditInfoModalProps) {
  const [address, setAddress] = useState("");
  const [owner, setOwner] = useState("");
  const [description, setDescription] = useState("");
  
  // Package Information States
  const [packDate, setPackDate] = useState("");
  const [packWeight, setPackWeight] = useState("");
  const [productFrom, setProductFrom] = useState("");
  const [packerShipper, setPackerShipper] = useState("");
  const [supplierIdTracking, setSupplierIdTracking] = useState("");
  
  const [isSaving, setIsSaving] = useState(false);
  const [geoError, setGeoError] = useState("");

  // Reset fields when modal opens
  useEffect(() => {
    if (isOpen) {
      setAddress(currentAddress || "");
      setOwner(currentOwner || "");
      setDescription(currentDescription || "");
      
      // Initialize package information
      setPackDate(currentPackDate || "");
      setPackWeight(currentPackWeight || "");
      setProductFrom(currentProductFrom || "");
      setPackerShipper(currentPackerShipper || "");
      setSupplierIdTracking(currentSupplierIdTracking || "");
    }
  }, [isOpen, currentAddress, currentOwner, currentDescription, 
      currentPackDate, currentPackWeight, currentProductFrom, 
      currentPackerShipper, currentSupplierIdTracking]);

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
    if (!boxId || !address.trim() || !owner.trim() || 
        !packDate.trim() || !packWeight.trim() || !productFrom.trim() ||
        !packerShipper.trim() || !supplierIdTracking.trim()) {
      toast.error("Please fill in all required fields");
      return;
    }

    setIsSaving(true);
    setGeoError("");

    try {
      const coords = await geocodeAddress(address.trim());

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
        supplierIdTracking: supplierIdTracking.trim()
      };
      
      onSave(
        boxId,
        finalAddress,
        owner.trim(),
        address.trim(),
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
    setOwner(currentOwner || "");
    setDescription(currentDescription || "");
    setPackDate(currentPackDate || "");
    setPackWeight(currentPackWeight || "");
    setProductFrom(currentProductFrom || "");
    setPackerShipper(currentPackerShipper || "");
    setSupplierIdTracking(currentSupplierIdTracking || "");
    onClose();
  };

  return (
    <Dialog open={isOpen} onOpenChange={onClose}>
      <DialogContent className="sm:max-w-lg max-h-[90vh] p-0 flex flex-col">
        <DialogHeader className="px-6 pt-6 pb-0">
          <DialogTitle className="text-xl font-bold text-gray-900 flex items-center gap-2">
            <div className="p-2 bg-blue-600 rounded-md border-2 border-blue-700">
              <User className="h-5 w-5 text-white" />
            </div>
            Edit Tracking Box Info
          </DialogTitle>
        </DialogHeader>

        <ScrollArea className="flex-1 overflow-auto">
          <div className="space-y-6 p-6">
            {boxId && (
              <div className="text-center space-y-2">
                <div className="inline-flex items-center px-3 py-1 rounded-md text-sm font-medium bg-gray-200 text-gray-800 border border-gray-400">
                  Box ID: {boxId.toUpperCase()}
                </div>
                {referenceCode && (
                  <div className="text-xs text-gray-600 font-mono">
                    Reference Code: {referenceCode}
                  </div>
                )}
              </div>
            )}

            {/* Owner Field */}
            <div className="space-y-1">
              <label className="flex items-center gap-2 text-sm font-bold text-gray-700">
                <User className="h-4 w-4 text-purple-600" />
                Owner
              </label>
              <input
                type="text"
                value={owner}
                onChange={(e) => setOwner(e.target.value)}
                placeholder="Enter the owner's name..."
                className="w-full p-3 border-2 border-gray-300 rounded-md text-sm"
                maxLength={50}
              />
              <div className="text-xs text-gray-400">{owner.length}/50</div>
            </div>

            {/* Description Field */}
            <div className="space-y-1">
              <label className="flex items-center gap-2 text-sm font-bold text-gray-700">
                <FileText className="h-4 w-4 text-slate-600" />
                Item Description
              </label>
              <textarea
                value={description}
                onChange={(e) => setDescription(e.target.value)}
                placeholder="Add a description for this tracking box..."
                className="w-full p-3 border-2 border-gray-300 rounded-md resize-none h-24 text-sm"
                maxLength={500}
              />
              <div className="text-xs text-gray-400">
                {description.length}/500
              </div>
            </div>

            {/* Address Field */}
            <div className="space-y-1">
              <label className="flex items-center gap-2 text-sm font-bold text-gray-700">
                <MapPin className="h-4 w-4 text-blue-600" />
                Address
              </label>
              <div className="relative">
                <textarea
                  value={address}
                  onChange={(e) => setAddress(e.target.value)}
                  placeholder="Enter the shipping address..."
                  className="w-full p-3 border-2 border-gray-300 rounded-md resize-none h-20 text-sm"
                  maxLength={200}
                />
                <div className="absolute bottom-2 right-2 text-xs text-gray-400">
                  {address.length}/200
                </div>
              </div>
              {geoError && (
                <div className="bg-red-50 border border-red-200 rounded-lg p-3">
                  <p className="text-red-700 text-sm">{geoError}</p>
                </div>
              )}
            </div>

            {/* Package Information Section */}
            <div className="space-y-4 border-t-2 border-gray-200 pt-4 mt-6">
              <h3 className="text-lg font-bold text-gray-800 flex items-center gap-2">
                <Package className="h-5 w-5 text-orange-600" />
                Package Information
              </h3>
              
              {/* Pack Date Field */}
              <div className="space-y-1">
                <label className="flex items-center gap-2 text-sm font-bold text-gray-700">
                  <Calendar className="h-4 w-4 text-green-600" />
                  Pack Date <span className="text-red-500">*</span>
                </label>
                <input
                  type="date"
                  value={packDate}
                  onChange={(e) => setPackDate(e.target.value)}
                  className="w-full p-3 border-2 border-gray-300 rounded-md text-sm"
                  required
                />
              </div>

              {/* Pack Weight Field */}
              <div className="space-y-1">
                <label className="flex items-center gap-2 text-sm font-bold text-gray-700">
                  <Weight className="h-4 w-4 text-yellow-600" />
                  Pack Weight <span className="text-red-500">*</span>
                </label>
                <input
                  type="text"
                  value={packWeight}
                  onChange={(e) => setPackWeight(e.target.value)}
                  placeholder="e.g., 5.2 kg, 10 lbs"
                  className="w-full p-3 border-2 border-gray-300 rounded-md text-sm"
                  maxLength={50}
                  required
                />
                <div className="text-xs text-gray-400">{packWeight.length}/50</div>
              </div>

              {/* Product From Field */}
              <div className="space-y-1">
                <label className="flex items-center gap-2 text-sm font-bold text-gray-700">
                  <MapPin className="h-4 w-4 text-red-600" />
                  Product From <span className="text-red-500">*</span>
                </label>
                <input
                  type="text"
                  value={productFrom}
                  onChange={(e) => setProductFrom(e.target.value)}
                  placeholder="Origin or source location"
                  className="w-full p-3 border-2 border-gray-300 rounded-md text-sm"
                  maxLength={100}
                  required
                />
                <div className="text-xs text-gray-400">{productFrom.length}/100</div>
              </div>

              {/* Packer/Shipper Field */}
              <div className="space-y-1">
                <label className="flex items-center gap-2 text-sm font-bold text-gray-700">
                  <Truck className="h-4 w-4 text-indigo-600" />
                  Packer / Shipper <span className="text-red-500">*</span>
                </label>
                <input
                  type="text"
                  value={packerShipper}
                  onChange={(e) => setPackerShipper(e.target.value)}
                  placeholder="Name of packer or shipping company"
                  className="w-full p-3 border-2 border-gray-300 rounded-md text-sm"
                  maxLength={100}
                  required
                />
                <div className="text-xs text-gray-400">{packerShipper.length}/100</div>
              </div>

              {/* Supplier ID/Tracking Field */}
              <div className="space-y-1">
                <label className="flex items-center gap-2 text-sm font-bold text-gray-700">
                  <Hash className="h-4 w-4 text-purple-600" />
                  Supplier ID# / Tracking # <span className="text-red-500">*</span>
                </label>
                <input
                  type="text"
                  value={supplierIdTracking}
                  onChange={(e) => setSupplierIdTracking(e.target.value)}
                  placeholder="Supplier ID or tracking number"
                  className="w-full p-3 border-2 border-gray-300 rounded-md text-sm"
                  maxLength={100}
                  required
                />
                <div className="text-xs text-gray-400">{supplierIdTracking.length}/100</div>
              </div>
            </div>
          </div>
        </ScrollArea>

        {/* Action Buttons - Fixed at bottom */}
        <div className="px-6 pb-6 pt-4 border-t">
          <div className="flex gap-3">
            <Button
              onClick={handleCancel}
              variant="outline"
              className="flex-1 flex items-center gap-2 border-2 border-gray-400 rounded-md hover:bg-gray-100 hover:border-gray-500 text-gray-700"
            >
              <X className="h-4 w-4" />
              Cancel
            </Button>
            <Button
              onClick={handleSave}
              disabled={!address.trim() || !owner.trim() || !packDate.trim() || 
                       !packWeight.trim() || !productFrom.trim() || !packerShipper.trim() || 
                       !supplierIdTracking.trim() || isSaving}
              className="flex-1 flex items-center gap-2 bg-blue-600 hover:bg-blue-700 text-white border-2 border-blue-700 rounded-md disabled:opacity-50 disabled:cursor-not-allowed disabled:bg-gray-400 disabled:border-gray-500"
            >
              {isSaving ? (
                "Saving..."
              ) : (
                <>
                  <Save className="h-4 w-4" />
                  Save Changes
                </>
              )}
            </Button>
          </div>

          {/* Warning Text */}
          <p className="text-xs text-gray-500 text-center bg-gray-100 p-2 rounded-md border border-gray-300 mt-3">
            Note: Changes will be applied immediately and synced to the
            database.
          </p>
        </div>
      </DialogContent>
    </Dialog>
  );
}
