/*
    Copyright (c) 2013 Yaniv Kamay,
    All rights reserved.

    Source code is provided for evaluation purposes only. Modification or use in
    source code for any other purpose is prohibited.

    Binary code (i.e. the binary form of source code form) is allowed to use for
    evaluation purposes only. Modification or use in binary code for any other
    purpose is prohibited.

    Redistribution, in source form or in binary form, with or without modification,
    are not permitted.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
    ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTOR BE LIABLE FOR
    ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
    ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
    NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
    IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

DefinitionBlock("DSDT.aml",  // AML file name
                "DSDT",      // table signature
                2,           // compliance revision
                "OBJCX",     // OEM ID
                "NOXMB01",   // table ID
                1)           // OEM revision
{
    Name (_S0, Package (0x04) // working
    {
        0x00,   // value for PM1a_CNT.SLP_TYP
        0x00,   // value for PM1b_CNT.SLP_TYP 
        0x00,   // reserved 
        0x00,   // reserved
    })
    Name (_S1, Package (0x04) // sleeping with processor context maintained
    {
        0x01, 
        0x00, 
        0x00, 
        0x00,
    })
    Name (_S5, Package (0x04) // soft off
    {
        0x05, 
        0x00, 
        0x00, 
        0x00,
    })

    Scope(\_SB) // system bus
    {   
        Device(PCI0) // root PCI bus
        {
            Name(_HID, EISAID("PNP0A03"))
            Name(_ADR, 0) // bus 0
            Name(_UID, 0) // the _UID must be unique across all devices with
                          // either a common _HID or _CID
            Name(RBUF, ResourceTemplate()
            {
                WordBusNumber(ResourceProducer, MinFixed, MaxFixed, PosDecode,
                    0x0000, // granularity
                    0x0000, // minimum
                    0x00ff, // maximum
                    0x0000, // offset
                    0x0100, // length
                    ,, )
                IO(Decode16,
                    0x0cf8, // minimum
                    0x0cf8, // maximum
                    0x01,   // alignment
                    0x08,   // length
                    )
                WordIO(ResourceProducer, MinFixed, MaxFixed, PosDecode, EntireRange,
                    0x0000, // granularity
                    0x0000, // minimum
                    0x0cf7, // maximum
                    0x0000, // translation offset
                    0x0cf8, // length
                    ,, , TypeStatic)
                WordIO(ResourceProducer, MinFixed, MaxFixed, PosDecode, EntireRange,
                    0x0000, // granularity
                    0x0d00, // minimum
                    0xffff, // maximum
                    0x0000, // translation offset
                    0xf300, // length
                    ,, , TypeStatic)
                DWordMemory(ResourceProducer, PosDecode, MinFixed, MaxFixed,
                            NonCacheable, ReadWrite,
                    0x00000000, // granularity
                    0x000a0000, // minimum
                    0x000bffff, // maximum
                    0x00000000, // translation offset
                    0x00020000, // length
                    ,, , AddressRangeMemory, TypeStatic)
                DWordMemory(ResourceProducer, PosDecode, MinFixed, MaxFixed,
                            NonCacheable, ReadWrite,
                    0x00000000, // granularity
                    0xe0000000, // minimum
                    0xfebfffff, // maximum
                    0x00000000, // translation offset
                    0x1ec00000, // length
                    ,, PH32, AddressRangeMemory, TypeStatic)
                QWordMemory(ResourceProducer, PosDecode, MinFixed, MaxFixed,
                            NonCacheable, ReadWrite,
                    0x0000000000000000, // granularity
                    0x0000000100000000, // minimum
                    0x0000001fffffffff, // maximum
                    0x0000000000000000, // translation offset
                    0x0000001f00000000, // Length
                    ,, PH64, AddressRangeMemory, TypeStatic)   
            })

            Method(_CRS, 0, NotSerialized)
            {
                OperationRegion(PCIH, SystemMemory, ^HOST.ROOT, 32)

                Field(PCIH, AnyAcc, NoLock, Preserve)
                {
                    ST64, 64,
                    EN64, 64,
                    ST32, 32,
                }

                CreateDWordField(RBUF, ^PH32._MIN, H32S)
                CreateDWordField(RBUF, ^PH32._LEN, H32L)

                Store(ST32, Local0)
                Store(Local0, H32S)
                Subtract(0xfec00000, Local0, H32L)

                CreateQWordField(RBUF, ^PH64._MIN, H64S)
                CreateQWordField(RBUF, ^PH64._MAX, H64E)
                CreateQWordField(RBUF, ^PH64._LEN, H64L)

                Store(ST64, Local1)
                Store(EN64, Local2)

                Name(MSFT, "Microsoft")

                // the following is a workaround for BSoD while starting instalation of Win7
                if (LAnd(LEqual(StrN(MSFT, _OS, SizeOf(MSFT)), 1),
                                LGreater(Local2, 0x0000002000000000))) {

                    Store(0x0000002000000000, Local2)

                    if (LGreaterEqual(Local1, Local2)) {
                        Store(0, H64S)
                        Store(0, H64L)
                        Store(0, H64E)
                        Return (RBUF)
                    }  
                }

                Store(Local1, H64S)
                Subtract(Local2, Local1, H64L)
                Subtract(Local2, 1, H64E)

                Return (RBUF)
            }

            Name(_PRT, Package()
            { 
                // slot 0
                Package() { 0x0000ffff /* _ADR*/,
                            0 /*pin*/,
                            IL00 /*PCI interrupt link or 0 in case of hard wire*/,
                            0 /*in case of hard wire: global system interrupt number*/},
                Package() {0x0000ffff, 1, IL01, 0},
                Package() {0x0000ffff, 2, IL02, 0},
                Package() {0x0000ffff, 3, IL03, 0},
                
                // slot 1
                Package() {0x0001ffff, 0, IL01, 0},
                Package() {0x0001ffff, 1, IL02, 0},
                Package() {0x0001ffff, 2, IL03, 0},
                Package() {0x0001ffff, 3, IL04, 0},
                
                // Pslot 2
                Package() {0x0002ffff, 0, IL02, 0},
                Package() {0x0002ffff, 1, IL03, 0},
                Package() {0x0002ffff, 2, IL04, 0},
                Package() {0x0002ffff, 3, IL05, 0},
                
                // slot 3
                Package() {0x0003ffff, 0, IL03, 0},
                Package() {0x0003ffff, 1, IL04, 0},
                Package() {0x0003ffff, 2, IL05, 0},
                Package() {0x0003ffff, 3, IL06, 0},
                
                // slot 4
                Package() {0x0004ffff, 0, 0, 9}, // SCI is hard-wired to interrupt pin 9
                Package() {0x0004ffff, 1, IL05, 0},
                Package() {0x0004ffff, 2, IL06, 0},
                Package() {0x0004ffff, 3, IL07, 0},
                
                // slot 5
                Package() {0x0005ffff, 0, IL05, 0},
                Package() {0x0005ffff, 1, IL06, 0},
                Package() {0x0005ffff, 2, IL07, 0},
                Package() {0x0005ffff, 3, IL00, 0},
                
                // slot 6
                Package() {0x0006ffff, 0, IL06, 0},
                Package() {0x0006ffff, 1, IL07, 0},
                Package() {0x0006ffff, 2, IL00, 0},
                Package() {0x0006ffff, 3, IL01, 0},
                
                // slot 7
                Package() {0x0007ffff, 0, IL07, 0},
                Package() {0x0007ffff, 1, IL00, 0},
                Package() {0x0007ffff, 2, IL01, 0},
                Package() {0x0007ffff, 3, IL02, 0},
                
                // slot 8
                Package() {0x0008ffff, 0, IL00, 0},
                Package() {0x0008ffff, 1, IL01, 0},
                Package() {0x0008ffff, 2, IL02, 0},
                Package() {0x0008ffff, 3, IL03, 0},
                
                // slot 9
                Package() {0x0009ffff, 0, IL01, 0},
                Package() {0x0009ffff, 1, IL02, 0},
                Package() {0x0009ffff, 2, IL03, 0},
                Package() {0x0009ffff, 3, IL04, 0},
                
                // slot 10
                Package() {0x000affff, 0, IL02, 0},
                Package() {0x000affff, 1, IL03, 0},
                Package() {0x000affff, 2, IL04, 0},
                Package() {0x000affff, 3, IL05, 0},
                
                // slot 11
                Package() {0x000bffff, 0, IL03, 0},
                Package() {0x000bffff, 1, IL04, 0},
                Package() {0x000bffff, 2, IL05, 0},
                Package() {0x000bffff, 3, IL06, 0},
                
                // PCI Slot 12
                Package() {0x000cffff, 0, IL04, 0},
                Package() {0x000cffff, 1, IL05, 0},
                Package() {0x000cffff, 2, IL06, 0},
                Package() {0x000cffff, 3, IL07, 0},
                
                // slot 13
                Package() {0x000dffff, 0, IL05, 0},
                Package() {0x000dffff, 1, IL06, 0},
                Package() {0x000dffff, 2, IL07, 0},
                Package() {0x000dffff, 3, IL00, 0},
                
                // slot 14
                Package() {0x000effff, 0, IL06, 0},
                Package() {0x000effff, 1, IL07, 0},
                Package() {0x000effff, 2, IL00, 0},
                Package() {0x000effff, 3, IL01, 0},
                
                // slot 15
                Package() {0x000fffff, 0, IL07, 0},
                Package() {0x000fffff, 1, IL00, 0},
                Package() {0x000fffff, 2, IL01, 0},
                Package() {0x000fffff, 3, IL02, 0},
            })

            Device(HOST)
            {
                Name(_ADR, 0x00000000)
    
                OperationRegion(STER, PCI_Config, 0x40, 0x4)
                Mutex(STRL, 15 /*SyncLevel*/) // All Acquire terms must refer to a synchronization
                                              // object with an equal or greater SyncLevel to
                                              // current level, and all Release terms must refer to
                                              // a synchronization object with equal or lower
                                              // SyncLevel to the current level.
                                              //
                                              // 15 is the top SyncLevel
                Field(STER, ByteAcc, NoLock, Preserve)
                {
                    LINK, 8,
                    IRQN, 8,
                    STAT, 8,
                    DISA, 8,
                }

                OperationRegion(BIOS, PCI_Config, 0x44, 0x4)
                Field(BIOS, DWordAcc, NoLock, Preserve)
                {
                    ROOT, 32,
                }
            }

            Device(ISA)
            {
                Name(_ADR, 0x00010000)
                Name(_HID, EISAID("PNP0A00"))
                Name(_UID, 0)

                Device(PIC)
                {
                    Name(_HID, EISAID("PNP0000"))

                    Name(_CRS, ResourceTemplate()
                    {
                        IO(Decode16,
                           0x0020,  // minimum
                           0x0020,  // maximum
                           0x01,    // alignment
                           0x02,    // length
                        )
                        IO(Decode16,
                           0x00a0,  // minimum
                           0x00a0,  // maximum
                           0x01,    // alignment
                           0x02,    // length
                        )
                        IRQNoFlags() {2}
                    })
                }

                Device(PIT0)
                {
                    Name(_HID, EISAID("PNP0100"))
                    Name(_CRS, ResourceTemplate()
                    {
                        IO(Decode16,
                           0x0040,  // minimum
                           0x0040,  // maximum
                           0x00,    // alignment
                           0x04,    // length
                        )
                        IRQNoFlags() {0}
                    })
                }

                Device(RTC)
                {
                    Name(_HID, EISAID("PNP0B00"))
                    Name(_CRS, ResourceTemplate()
                    {
                        IO(Decode16,
                           0x0070,  // minimum
                           0x0070,  // maximum
                           0x00,    // alignment
                           0x04,    // length
                        )
                        IRQNoFlags() {8}
                    })

                    Name(ATT1, ResourceTemplate()
                    {
                        IO(Decode16,
                           0x0070,  // minimum
                           0x0070,  // maximum
                           0x00,    // alignment
                           0x04,    // length
                        )
                    })
                }

                Device(SPKR)
                {
                    Name(_HID, EISAID("PNP0800"))
                    Name(_CRS, ResourceTemplate()
                    {
                        IO(Decode16,
                           0x0061,  // minimum
                           0x0061,  // maximum
                           0x01,    // alignment
                           0x01,    // length
                        )
                    })
                }

                Device(PS2K)
                {
                    Name(_HID, EISAID("PNP0303"))

                    Method(_STA, 0, NotSerialized)
                    {
                        Return (0x0b)
                    }
    
                    Name(_CRS, ResourceTemplate()
                    {
                        IO(Decode16,
                           0x0060,  // minimum
                           0x0060,  // maximum
                           0x01,    // alignment
                           0x01,    // length
                        )
                        IO(Decode16,
                           0x0064,  // minimum
                           0x0064,  // maximum
                           0x01,    // alignment
                           0x01,    // length
                        )
                        IRQNoFlags() {1}
                    })
                }
    
                Device(PS2M)
                {
                    Name(_HID, EISAID("PNP0F13"))

                    Method(_STA, 0, NotSerialized)
                    {
                            Return (0x0b)
                    }
    
                    Name(_CRS, ResourceTemplate()
                    {
                        /*
                        IO(Decode16,
                           0x0060,  // minimum
                           0x0060,  // maximum
                           0x01,    // alignment
                           0x01,    // length
                        )
                        IO(Decode16,
                           0x0064,  // minimum
                           0x0064,  // maximum
                           0x01,    // alignment
                           0x01,    // length
                           )
                        */
                        IRQNoFlags() {12}
                    })
                }
            }
        }
    }
    
    Scope(\_SB.PCI0)
    {
        Name(ILRT, ResourceTemplate()
        {
            IRQ (Level, ActiveLow, Shared) { 3, 4, 5, 6, 7, 9, 10, 11}
        })

        Method(STA, 1, NotSerialized)
        {
            // Arg0 is link id

            Acquire(^HOST.STRL, 0xffff) // TimeoutValue of 0xffff (or greater) indicates that
                                        // there is no timeout and the operation will wait
                                        // indefinitely.

            Store(Arg0 , ^HOST.LINK)
            Store(^HOST.STAT, Local0)
            And(Local0, 0x03, Local0) // bit 0 error; bit 1 enabled

            Release(^HOST.STRL)

            If (LEqual(Local0, 0x02)) {
                Return (0x0b) // enabled
            } Else {
                Return (0x09)
            }
        }

        Method(DIS, 1, NotSerialized)
        {
            // Arg0 is link id

            Acquire(^HOST.STRL, 0xffff)

            Store(Arg0, ^HOST.LINK)
            Store(^HOST.STAT, Local0)
            And(Local0, 0x03, Local1)

            If (LEqual(Local1, 0x02)) {
                // no error and device is enabled
                Store(0, ^HOST.DISA)
            }

            Release(^HOST.STRL)
        }

        Method(CRS, 1, NotSerialized)
        {
            // Arg0 is link id

            Acquire(^HOST.STRL, 0xffff)

            Store(ILRT, Local0)
            CreateWordField(Local0, 0x01, MASK)
            Store(0, MASK)

            Store(Arg0, ^HOST.LINK)
            Store(^HOST.STAT, Local1)
            And(Local1, 0x01, Local1)

            If (LEqual(Local1, 0x00)) { // no error
                ShiftLeft(0x01, ^HOST.IRQN, MASK)
            }

            Release(^HOST.STRL)

            Return (Local0)
        }


        Method(SRS, 2, NotSerialized)
        {
            // Arg0 is link id
            // Arg1 is IRQ Descriptor

            Acquire(^HOST.STRL, 0xffff)

            Store(Arg0, ^HOST.LINK)
            Store(^HOST.STAT, Local0)
            And(Local0, 0x01, Local0)

            If (LEqual(Local0, 0x00)) { // no error
                CreateWordField(Arg1, 0x01, IRQM)
                FindSetRightBit(IRQM, Local1)
                Decrement(Local1)
                Store(Local1, ^HOST.IRQN)
            }

            Release(^HOST.STRL)
        }

        Device(IL00) // PCI interrupt link
        {
            Name(_HID, EISAID("PNP0C0F"))
            Name(_UID, 0)

            //These objects have _PRS, _CRS, _SRS, and _DIS

            Method(_PRS, 0, NotSerialized) // possible resource settings
            {
                Return (ILRT)
            }

            Method(_STA, 0, NotSerialized) // status
            {
                Return (STA(0))
            }

            Method(_DIS, 0, NotSerialized) // disable
            {
                DIS(0)
            }

            Method(_CRS, 0, NotSerialized) // current resource settings
            {
                Return (CRS(0))
            }

            Method(_SRS, 1, NotSerialized) // set resource settings; if the device is disabled,
                                           // _SRS enables the device at the specified resources. 
            {
                // Arg0 is IRQ Descriptor

                SRS(0, Arg0)
            }
        }

        Device(IL01)
        {
            Name(_HID, EISAID("PNP0C0F"))
            Name(_UID, 1)

            Method(_PRS, 0, NotSerialized)
            {
                Return (ILRT)
            }

            Method(_STA, 0, NotSerialized)
            {
                Return (STA(1))
            }

            Method(_DIS, 0, NotSerialized)
            {
                DIS(1)
            }

            Method(_CRS, 0, NotSerialized)
            {
                Return (CRS(1))
            }

            Method(_SRS, 1, NotSerialized)
            {
                SRS(1, Arg0)
            }
        }

        Device(IL02)
        {
            Name(_HID, EISAID("PNP0C0F"))
            Name(_UID, 2)

            Method(_PRS, 0, NotSerialized)
            {
                Return (ILRT)
            }

            Method(_STA, 0, NotSerialized)
            {
                Return (STA(2))
            }

            Method(_DIS, 0, NotSerialized)
            {
                DIS(2)
            }

            Method(_CRS, 0, NotSerialized)
            {
                Return (CRS(2))
            }

            Method(_SRS, 1, NotSerialized)
            {
                SRS(2, Arg0)
            }
        }

        Device(IL03)
        {
            Name(_HID, EISAID("PNP0C0F"))
            Name(_UID, 3)

            Method(_PRS, 0, NotSerialized)
            {
                Return (ILRT)
            }

            Method(_STA, 0, NotSerialized)
            {
                Return (STA(3))
            }

            Method(_DIS, 0, NotSerialized)
            {
                DIS(3)
            }

            Method(_CRS, 0, NotSerialized)
            {
                Return (CRS(3))
            }

            Method(_SRS, 1, NotSerialized)
            {
                SRS(3, Arg0)
            }
        }

        Device(IL04)
        {
            Name(_HID, EISAID("PNP0C0F"))
            Name(_UID, 4)

            Method(_PRS, 0, NotSerialized)
            {
                Return (ILRT)
            }

            Method(_STA, 0, NotSerialized)
            {
                Return (STA(4))
            }

            Method(_DIS, 0, NotSerialized)
            {
                DIS(4)
            }

            Method(_CRS, 0, NotSerialized)
            {
                Return (CRS(4))
            }

            Method(_SRS, 1, NotSerialized)
            {
                SRS(4, Arg0)
            }
        }

        Device(IL05)
        {
            Name(_HID, EISAID("PNP0C0F"))
            Name(_UID, 5)

            Method(_PRS, 0, NotSerialized)
            {
                Return (ILRT)
            }

            Method(_STA, 0, NotSerialized)
            {
                Return (STA(5))
            }

            Method(_DIS, 0, NotSerialized)
            {
                DIS(5)
            }

            Method(_CRS, 0, NotSerialized)
            {
                Return (CRS(5))
            }

            Method(_SRS, 1, NotSerialized)
            {
                SRS(5, Arg0)
            }
        }

        Device(IL06)
        {
            Name(_HID, EISAID("PNP0C0F"))
            Name(_UID, 6)

            Method(_PRS, 0, NotSerialized)
            {
                Return (ILRT)
            }

            Method(_STA, 0, NotSerialized)
            {
                Return (STA(6))
            }

            Method(_DIS, 0, NotSerialized)
            {
                DIS(6)
            }

            Method(_CRS, 0, NotSerialized)
            {
                Return (CRS(6))
            }

            Method(_SRS, 1, NotSerialized)
            {
                SRS(6, Arg0)
            }
        }

        Device(IL07)
        {
            Name(_HID, EISAID("PNP0C0F"))
            Name(_UID, 7)

            Method(_PRS, 0, NotSerialized)
            {
                Return (ILRT)
            }

            Method(_STA, 0, NotSerialized)
            {
                Return (STA(7))
            }

            Method(_DIS, 0, NotSerialized)
            {
                DIS(7)
            }

            Method(_CRS, 0, NotSerialized)
            {
                Return (CRS(7))
            }

            Method(_SRS, 1, NotSerialized)
            {
                SRS(7, Arg0)
            }
        }
    }

    Method(StrN, 3, NotSerialized)
    {
        If (LOr(LLess(SizeOf(Arg0), Arg2), LLess(SizeOf(Arg1), Arg2))) {
            Return (0)
        }

        Name (STR0, Buffer (Add(SizeOf(Arg0), 1)) {})
        Name (STR1, Buffer (Add(SizeOf(Arg1), 1)) {})
        Store (Arg0, STR0)
        Store (Arg1, STR1)

        Store(0, Local0)

        While (LLess(Local0, Arg2)) {
            Store(DerefOf(Index(STR0, Local0)), Local1)
            Store(DerefOf(Index(STR1, Local0)), Local2)

            If (LNotEqual(Local1, Local2)) {
                Return (0)
            }

            Increment(Local0)
        }

        Return (1)
    }
}

