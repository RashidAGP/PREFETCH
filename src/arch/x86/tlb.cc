/*
 * Copyright (c) 2007-2008 The Hewlett-Packard Development Company
 * All rights reserved.
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "arch/x86/tlb.hh"

#include <cstring>
#include <memory>

#include "arch/x86/faults.hh"
#include "arch/x86/insts/microldstop.hh"
#include "arch/x86/pagetable_walker.hh"
#include "arch/x86/pseudo_inst_abi.hh"
#include "arch/x86/regs/misc.hh"
#include "arch/x86/regs/msr.hh"
#include "arch/x86/x86_traits.hh"
#include "base/trace.hh"
#include "cpu/thread_context.hh"
#include "debug/TLB.hh"
#include "mem/packet_access.hh"
#include "mem/page_table.hh"
#include "mem/request.hh"
#include "sim/full_system.hh"
#include "sim/process.hh"
#include "sim/pseudo_inst.hh"

namespace gem5
{

namespace X86ISA {

TLB::TLB(const Params &p)
    : BaseTLB(p), configAddress(0), size(p.size),l2_tlb_size(p.l2_tlb_size),
      l1_way(p.l1_way),l2_tlb_assoc(p.l2_tlb_assoc),
      tlb(p.size),l2tlb(p.l2_tlb_size), lruSeq(0),lruSeq_l2(0), m5opRange(p.system->m5opRange()), stats(this)
{
    if (!size)
        fatal("TLBs must have a non-zero size.\n");
    size =p.size;
    for (int x = 0; x < size; x++) {
        tlb[x].trieHandle = NULL;
        freeList.push_back(&tlb[x]);
    }

    l2tlb.resize(l2_tlb_size);
    for (int x = 0; x < l2_tlb_size; x++) {
        l2tlb[x].trieHandle = NULL;
        freeList_l2.push_back(&l2tlb[x]);
    }
    if (p.l2_tlb_size >= 0) {
        l2_tlb_size = (uint32_t) p.l2_tlb_size;
        l2_tlb_assoc = (uint32_t) p.l2_tlb_assoc;
        DPRINTF(TLB, "L2 TLB Initialization: size-%u assoc-%u\n", l2_tlb_size,
              l2_tlb_assoc);
    }
    walker = p.walker;
    walker->setTLB(this);
}

void
TLB::evictLRU_l1(Addr vpn)
{
    // Find the entry with the lowest (and hence least recently updated)
    // sequence number.

    DPRINTF(TLB, "REMOVE from L1. Insert for VPN:%#x\n",vpn);
    uint32_t set = vpn % (size/l1_way);
    uint32_t start = set * l1_way;
    uint32_t end = (set + 1) * (l1_way);
    unsigned lru = 0;
    for (unsigned i = start; i < end; i++) {
        if (tlb[i].lruSeq < tlb[lru].lruSeq)
            lru = i;
    }
    // UAC
    if (tlb[lru].logBytes == 12){
        add_page_eviction_l1_4kb(tlb[lru].vaddr, tlb[lru].paddr);
        //add_page_eviction_l1_4kb_VA(tlb[lru].vaddr);
    }else{
        add_page_eviction_l1_2mb(tlb[lru].vaddr, tlb[lru].paddr);
        //add_page_eviction_l1_2mb_VA(tlb[lru].vaddr);
    }
    print_eviction();
    // End UAC
    assert(tlb[lru].trieHandle);
    trie.remove(tlb[lru].trieHandle);
    tlb[lru].trieHandle = NULL;
    freeList.push_back(&tlb[lru]);
}

// L2 -------- Evict
// PA
void
TLB::add_page_eviction_l1_4kb(Addr address_v, Addr address_p){
	page_eviction_l1_4kb[address_v][address_p] = page_eviction_l1_4kb[address_v][address_p] + 1;
	//page_eviction_l1_time[address_v][address_p] = this->walker->curCycle();
}
void
TLB::add_page_eviction_l1_2mb(Addr address_v,Addr address_p){
	page_eviction_l1_2mb[address_v][address_p] = page_eviction_l1_2mb[address_v][address_p] + 1;
	//page_eviction_l1_time[address_t] = this->walker->curCycle();
}
// VA
void
TLB::add_page_eviction_l1_4kb_VA(Addr address_t){
	page_eviction_l1_4kb_VA[address_t] = page_eviction_l1_4kb_VA[address_t] + 1;
	page_eviction_l1_time_VA[address_t] = this->walker->curCycle();
}
void
TLB::add_page_eviction_l1_2mb_VA(Addr address_t){
	page_eviction_l1_2mb_VA[address_t] = page_eviction_l1_2mb_VA[address_t] + 1;
	page_eviction_l1_time_VA[address_t] = this->walker->curCycle();
}
void
TLB::evictLRU_l2(Addr vpn)
{
    // Find the entry with the lowest (and hence least recently updated)
    // sequence number.

    DPRINTF(TLB, "REMOVE from L2:%#x\n",vpn);
    uint32_t set = vpn % (l2_tlb_size/l2_tlb_assoc);
    uint32_t start = set * l2_tlb_assoc;
    uint32_t end = (set + 1) * (l2_tlb_assoc);
    unsigned lru = 0;
    for (unsigned i = start; i < end; i++) {
        if (l2tlb[i].lruSeq < l2tlb[lru].lruSeq)
            lru = i;
    }
    // UAC
    if (l2tlb[lru].logBytes == 12){
        add_page_eviction_l2_4kb(l2tlb[lru].vaddr, l2tlb[lru].paddr);
        //add_page_eviction_l2_4kb_VA(l2tlb[lru].vaddr);
    }else{
        add_page_eviction_l2_2mb(l2tlb[lru].vaddr, l2tlb[lru].paddr);
        //add_page_eviction_l2_2mb_VA(l2tlb[lru].vaddr);
    }
    // End UAC

    assert(l2tlb[lru].trieHandle);
    triel2.remove(l2tlb[lru].trieHandle);
    l2tlb[lru].trieHandle = NULL;
    freeList_l2.push_back(&l2tlb[lru]);
}
// RA
// L1 --------- insert
TlbEntry *
TLB::insert_l1(Addr vaddr, const TlbEntry &entry,uint64_t pc_id)
{

    DPRINTF(TLB,"Insert in L1.VA:%#x\n",vaddr);
    Addr vpn = 0;
    if (entry.logBytes == 12){
       vpn = vaddr >> 12;
    }else if (entry.logBytes == 21){
       vpn = vaddr >> 21;
    }

    TlbEntry *newEntry = trie.lookup(vaddr);
    if (newEntry) {
        assert(newEntry->vaddr == vaddr);
        return newEntry;
    }

    if (freeList.empty()){
        evictLRU_l1(vpn);
    }
    newEntry = freeList.front();
    freeList.pop_front();

    *newEntry = entry;
    newEntry->lruSeq = nextSeq();
    newEntry->vaddr = vaddr;
    newEntry->trieHandle =
    trie.insert(vaddr, TlbEntryTrie::MaxBits - entry.logBytes, newEntry);
    return newEntry;
}



// L2 --------- insert


TlbEntry *
TLB::insert_l2(Addr vaddr, Addr pa, Addr va_csv, const TlbEntry &entry,uint64_t pc_id)
{
  if (l2_tlb_size != 0){
	if (this->name() == "system.cpu.mmu.dtb"){
		   // Rashid L2 TLB Miss
		   DPRINTF(TLB,"Insert in L2 TLB. VA:%#x.\n",vaddr);
                   std::string cache_level_one_d = "system.ruby.l1_cntrl0.L1Dcache";
                   std::string cache_level_two = "system.ruby.l2_cntrl0.L2cache";
		   std::vector<SimObject *> simObjectList = SimObject::getSimObjectList();
		   gem5::ruby::CacheMemory* cache_level_prediction;
		   bool lookup_result_l1 = true;
		   bool lookup_result_l2 = true;
		   Addr paddr_to_check = entry.paddr | (vaddr & mask(entry.logBytes));
		   Addr line_paddr = 0;
		   line_paddr = pa >> 6;
		   line_paddr = line_paddr << 6;
		   DPRINTF(TLB,"L2 TLB Miss:%#x.\n",line_paddr);
		   bool inclusive = false;
		   for (SimObject* simObject : simObjectList) {
			if (simObject->name() == cache_level_one_d){
				cache_level_prediction = dynamic_cast<gem5::ruby::CacheMemory *>(simObject);
				DPRINTF(TLB,"Name is :%s.\n",cache_level_one_d);
                		stats.L2TLB_l1_access++;
				if (cache_level_prediction->lookup_rashid(line_paddr) == true){
					stats.L2TLB_l1_hit++;
					inclusive = true;
				}else{
					stats.L2TLB_l1_miss++;
				}
			}
		   }
		   for (SimObject* simObject : simObjectList) {
			if (simObject->name() == cache_level_two){
				cache_level_prediction = dynamic_cast<gem5::ruby::CacheMemory *>(simObject);
				DPRINTF(TLB,"Name is :%s.\n",cache_level_two);
                 		stats.L2TLB_l2_access++;
				if (cache_level_prediction->lookup_rashid(line_paddr) == true){
					stats.L2TLB_l2_hit++;
				}else{
					stats.L2TLB_l2_miss++;
					if (inclusive == true){
						//assert(inclusive && "Whyyyyyy \n");
					}
				
				}
			}
		}
	}
    vaddr = concAddrPcid(vaddr, pc_id);

    Addr vpn = 0;
    if (entry.logBytes == 12){
        vpn = vaddr >> 12;
    }else if (entry.logBytes == 21){
        vpn = vaddr >> 21;
    }
    if (entry.logBytes == 12){
	vaddr = vpn << 12;
    }else {
vaddr = vpn << 21;
}
    TlbEntry *newEntry = triel2.lookup(vaddr);
    if (newEntry) {
        assert(newEntry->vaddr == vaddr);
        return newEntry;
    }

    if (freeList_l2.empty())
        evictLRU_l2(vpn);

    newEntry = freeList_l2.front();
    freeList_l2.pop_front();

    *newEntry = entry;
    newEntry->lruSeq = nextSeq_l2();
    newEntry->vaddr = vaddr;
    if (FullSystem) {
        newEntry->trieHandle =
        triel2.insert(vaddr, TlbEntryTrie::MaxBits-entry.logBytes, newEntry);
    }
    else {
        newEntry->trieHandle =
        triel2.insert(vaddr, TlbEntryTrie::MaxBits, newEntry);
    }
    return newEntry;
  } else return NULL;

}
// End RA.
TlbEntry *
TLB::lookup_l1(Addr va, bool update_lru)
{
    TlbEntry *entry = trie.lookup(va);
    if (entry && update_lru)
        entry->lruSeq = nextSeq();
    return entry;
}


TlbEntry *
TLB::lookup_l2(Addr va, bool update_lru)
{
    TlbEntry *entry = triel2.lookup(va);
    if (entry && update_lru)
        entry->lruSeq = nextSeq_l2();
    return entry;
}

void
TLB::flushAll()
{
    DPRINTF(TLB, "Invalidating all entries.\n");
    for (unsigned i = 0; i < size; i++) {
        if (tlb[i].trieHandle) {
            trie.remove(tlb[i].trieHandle);
            tlb[i].trieHandle = NULL;
            freeList.push_back(&tlb[i]);
      }
    }

    for (unsigned i = 0; i < l2_tlb_size; i++) {
        if (l2tlb[i].trieHandle) {
            triel2.remove(l2tlb[i].trieHandle);
            l2tlb[i].trieHandle = NULL;
            freeList_l2.push_back(&l2tlb[i]);
      }
    }
}

void
TLB::setConfigAddress(uint32_t addr)
{
    configAddress = addr;
}

void
TLB::flushNonGlobal()
{
    DPRINTF(TLB, "Invalidating all non global entries.\n");
    for (unsigned set = 0; set < size; set++) {
        if (tlb[set].trieHandle && !tlb[set].global) {
            trie.remove(tlb[set].trieHandle);
            tlb[set].trieHandle = NULL;
            freeList.push_back(&tlb[set]);
        }
    }

    for (unsigned set = 0; set < l2_tlb_size; set++) {
        if (l2tlb[set].trieHandle && !l2tlb[set].global) {
            triel2.remove(l2tlb[set].trieHandle);
            l2tlb[set].trieHandle = NULL;
            freeList_l2.push_back(&l2tlb[set]);
        }
    }
}

void
TLB::demapPage(Addr va, uint64_t asn)
{
    TlbEntry *entry = NULL;
    entry = trie.lookup(va);
    if (entry != NULL) {
        trie.remove(entry->trieHandle);
        entry->trieHandle = NULL;
        freeList.push_back(entry);

        DPRINTF(TLB, "DEMAP L1:%#x\n",va);
    }
    TlbEntry *entry2 = NULL;
    entry2 = triel2.lookup(va);
    if (entry2 != NULL) {
        triel2.remove(entry2->trieHandle);
        entry2->trieHandle = NULL;
        freeList_l2.push_back(entry2);

        DPRINTF(TLB, "DEMAP L2:%#x\n",va);
    }

}

namespace
{

Cycles
localMiscRegAccess(bool read, RegIndex regNum,
                   ThreadContext *tc, PacketPtr pkt)
{
    if (read) {
        RegVal data = htole(tc->readMiscReg(regNum));
        assert(pkt->getSize() <= sizeof(RegVal));
        pkt->setData((uint8_t *)&data);
    } else {
        RegVal data = htole(tc->readMiscRegNoEffect(regNum));
        assert(pkt->getSize() <= sizeof(RegVal));
        pkt->writeData((uint8_t *)&data);
        tc->setMiscReg(regNum, letoh(data));
    }
    return Cycles(1);
}

} // anonymous namespace

Fault
TLB::translateInt(bool read, RequestPtr req, ThreadContext *tc)
{
    DPRINTF(TLB, "Addresses references internal memory.\n");
    Addr vaddr = req->getVaddr();
    Addr prefix = (vaddr >> 3) & IntAddrPrefixMask;
    if (prefix == IntAddrPrefixCPUID) {
        panic("CPUID memory space not yet implemented!\n");
    } else if (prefix == IntAddrPrefixMSR) {
        vaddr = (vaddr >> 3) & ~IntAddrPrefixMask;

        RegIndex regNum;
        if (!msrAddrToIndex(regNum, vaddr))
            return std::make_shared<GeneralProtection>(0);

        req->setPaddr(req->getVaddr());
        req->setLocalAccessor(
            [read,regNum](ThreadContext *tc, PacketPtr pkt)
            {
                return localMiscRegAccess(read, regNum, tc, pkt);
            }
        );

        return NoFault;
    } else if (prefix == IntAddrPrefixIO) {
        // TODO If CPL > IOPL or in virtual mode, check the I/O permission
        // bitmap in the TSS.

        Addr IOPort = vaddr & ~IntAddrPrefixMask;
        // Make sure the address fits in the expected 16 bit IO address
        // space.
        assert(!(IOPort & ~0xFFFF));
        if (IOPort == 0xCF8 && req->getSize() == 4) {
            req->setPaddr(req->getVaddr());
            req->setLocalAccessor(
                [read](ThreadContext *tc, PacketPtr pkt)
                {
                    return localMiscRegAccess(
                            read, misc_reg::PciConfigAddress, tc, pkt);
                }
            );
        } else if ((IOPort & ~mask(2)) == 0xCFC) {
            req->setFlags(Request::UNCACHEABLE | Request::STRICT_ORDER);
            Addr configAddress =
                tc->readMiscRegNoEffect(misc_reg::PciConfigAddress);
            if (bits(configAddress, 31, 31)) {
                req->setPaddr(PhysAddrPrefixPciConfig |
                        mbits(configAddress, 30, 2) |
                        (IOPort & mask(2)));
            } else {
                req->setPaddr(PhysAddrPrefixIO | IOPort);
            }
        } else {
            req->setFlags(Request::UNCACHEABLE | Request::STRICT_ORDER);
            req->setPaddr(PhysAddrPrefixIO | IOPort);
        }
        return NoFault;
    } else {
        panic("Access to unrecognized internal address space %#x.\n",
                prefix);
    }
}

Fault
TLB::finalizePhysical(const RequestPtr &req,
                      ThreadContext *tc, BaseMMU::Mode mode) const
{
    Addr paddr = req->getPaddr();

    if (m5opRange.contains(paddr)) {
        req->setFlags(Request::STRICT_ORDER);
        uint8_t func;
        pseudo_inst::decodeAddrOffset(paddr - m5opRange.start(), func);
        req->setLocalAccessor(
            [func, mode](ThreadContext *tc, PacketPtr pkt) -> Cycles
            {
                uint64_t ret;
                pseudo_inst::pseudoInst<X86PseudoInstABI, true>(tc, func, ret);
                if (mode == BaseMMU::Read)
                    pkt->setLE(ret);
                return Cycles(1);
            }
        );
    } else if (FullSystem) {
        // Check for an access to the local APIC
        LocalApicBase localApicBase =
            tc->readMiscRegNoEffect(misc_reg::ApicBase);
        AddrRange apicRange(localApicBase.base * PageBytes,
                            (localApicBase.base + 1) * PageBytes);

        if (apicRange.contains(paddr)) {
            // The Intel developer's manuals say the below restrictions apply,
            // but the linux kernel, because of a compiler optimization, breaks
            // them.
            /*
            // Check alignment
            if (paddr & ((32/8) - 1))
                return new GeneralProtection(0);
            // Check access size
            if (req->getSize() != (32/8))
                return new GeneralProtection(0);
            */
            // Force the access to be uncacheable.
            req->setFlags(Request::UNCACHEABLE | Request::STRICT_ORDER);
            req->setPaddr(x86LocalAPICAddress(tc->contextId(),
                                              paddr - apicRange.start()));
        }
    }

    return NoFault;
}

Fault
TLB::translate(const RequestPtr &req,
        ThreadContext *tc, BaseMMU::Translation *translation,
        BaseMMU::Mode mode, bool &delayedResponse, bool timing)
{
    Request::Flags flags = req->getFlags();
    int seg = flags & SegmentFlagMask;
    bool storeCheck = flags & Request::READ_MODIFY_WRITE;

    delayedResponse = false;

    // If this is true, we're dealing with a request to a non-memory address
    // space.
    if (seg == segment_idx::Ms) {
        return translateInt(mode == BaseMMU::Read, req, tc);
    }

    Addr vaddr = req->getVaddr();
    DPRINTF(TLB, "Translating vaddr %#x.\n", vaddr);

    HandyM5Reg m5Reg = tc->readMiscRegNoEffect(misc_reg::M5Reg);

    const Addr logAddrSize = (flags >> AddrSizeFlagShift) & AddrSizeFlagMask;
    const int addrSize = 8 << logAddrSize;
    const Addr addrMask = mask(addrSize);

    // If protected mode has been enabled...
    if (m5Reg.prot) {
        DPRINTF(TLB, "In protected mode.\n");
        // If we're not in 64-bit mode, do protection/limit checks
        if (m5Reg.mode != LongMode) {
            DPRINTF(TLB, "Not in long mode. Checking segment protection.\n");

            // CPUs won't know to use CS when building fetch requests, so we
            // need to override the value of "seg" here if this is a fetch.
            if (mode == BaseMMU::Execute)
                seg = segment_idx::Cs;

            SegAttr attr = tc->readMiscRegNoEffect(misc_reg::segAttr(seg));
            // Check for an unusable segment.
            if (attr.unusable) {
                DPRINTF(TLB, "Unusable segment.\n");
                return std::make_shared<GeneralProtection>(0);
            }
            bool expandDown = false;
            if (seg >= segment_idx::Es && seg <= segment_idx::Hs) {
                if (!attr.writable && (mode == BaseMMU::Write || storeCheck)) {
                    DPRINTF(TLB, "Tried to write to unwritable segment.\n");
                    return std::make_shared<GeneralProtection>(0);
                }
                if (!attr.readable && mode == BaseMMU::Read) {
                    DPRINTF(TLB, "Tried to read from unreadble segment.\n");
                    return std::make_shared<GeneralProtection>(0);
                }
                expandDown = attr.expandDown;

            }
            Addr base = tc->readMiscRegNoEffect(misc_reg::segBase(seg));
            Addr limit = tc->readMiscRegNoEffect(misc_reg::segLimit(seg));
            Addr offset;
            if (mode == BaseMMU::Execute)
                offset = vaddr - base;
            else
                offset = (vaddr - base) & addrMask;
            Addr endOffset = offset + req->getSize() - 1;
            if (expandDown) {
                DPRINTF(TLB, "Checking an expand down segment.\n");
                warn_once("Expand down segments are untested.\n");
                if (offset <= limit || endOffset <= limit)
                    return std::make_shared<GeneralProtection>(0);
            } else {
                if (offset > limit || endOffset > limit) {
                    DPRINTF(TLB, "Segment limit check failed, "
                            "offset = %#x limit = %#x.\n", offset, limit);
                    return std::make_shared<GeneralProtection>(0);
                }
            }
        }
        if (m5Reg.submode != SixtyFourBitMode && addrSize != 64)
            vaddr &= mask(32);
        // If paging is enabled, do the translation.
        if (m5Reg.paging) {
            DPRINTF(TLB, "Paging enabled.\n");
            // The vaddr already has the segment base applied.

            //Appending the pcid (last 12 bits of CR3) to the
            //page aligned vaddr if pcide is set
            CR4 cr4 = tc->readMiscRegNoEffect(misc_reg::Cr4);
            Addr pageAlignedVaddr = vaddr & (~mask(X86ISA::PageShift));
            CR3 cr3 = tc->readMiscRegNoEffect(misc_reg::Cr3);
            uint64_t pcid;

            if (cr4.pcide)
                pcid = cr3.pcid;
            else
                pcid = 0x000;

            pageAlignedVaddr = concAddrPcid(pageAlignedVaddr, pcid);
            TlbEntry *entry = lookup_l1(pageAlignedVaddr);

            if (mode == BaseMMU::Read) {
                stats.rdAccesses++;
            } else {
                stats.wrAccesses++;
            }
            if (!entry) {
                DPRINTF(TLB, "Handling a TLB miss for "
                        "address %#x at pc %#x.\n",
                        vaddr, tc->pcState().instAddr());
                if (mode == BaseMMU::Read) {
                    stats.rdMisses++;
                } else {
                    stats.wrMisses++;
                }
                entry =  lookup_l2(pageAlignedVaddr);
                stats.l2_tlb_Accesses++;
                if (entry){
                   DPRINTF(TLB,
                        "TLB Hit in L2 %#x at pc %#x.\n",
                        vaddr, tc->pcState().instAddr());
                   // Put L2 latency
                   Fault fault_l = NoFault;
                   auto event = new DelayedL2HitEvent(this,tc,translation,req,mode,fault_l,pcid,vaddr,*entry);
		   // Rashid L2 TLB Hit
                   std::string cache_level_one_d = "system.ruby.l1_cntrl0.L1Dcache";
                   std::string cache_level_two = "system.ruby.l2_cntrl0.L2cache";
		   std::vector<SimObject *> simObjectList = SimObject::getSimObjectList();
		   gem5::ruby::CacheMemory* cache_level_prediction;
		   bool lookup_result_l1 = true;
		   bool lookup_result_l2 = true;
		   Addr paddr_to_check = entry->paddr | (vaddr & mask(entry->logBytes));
		   Addr line_paddr = 0;
		   line_paddr = paddr_to_check >> 6;
		   line_paddr = line_paddr << 6;
		   stats.L1TLB_l1_access++;
		   stats.L1TLB_l2_access++;
		   DPRINTF(TLB,"L2 TLB Hit. PA:%#x, CL:%#x.\n",paddr_to_check,line_paddr);
		   bool inclusive = false;

                   if (this->name() == "system.cpu.mmu.dtb"){
		     for (SimObject* simObject : simObjectList) {
			if (simObject->name() == cache_level_one_d){
				cache_level_prediction = dynamic_cast<gem5::ruby::CacheMemory *>(simObject);
				if (cache_level_prediction->lookup_rashid(line_paddr) == true){
					stats.L1TLB_l1_hit++;
					inclusive = true;
				}else{
					stats.L1TLB_l1_miss++;
				}
			}else if (simObject->name() == cache_level_two){
				cache_level_prediction = dynamic_cast<gem5::ruby::CacheMemory *>(simObject);
				if (cache_level_prediction->lookup_rashid(line_paddr) == true){
					stats.L1TLB_l2_hit++;
				}else{
					stats.L1TLB_l2_miss++;
					assert(inclusive && "How it possible???\n");
				}
			}
		      }
		   }
		   //End Rashid L2 TLB Hit
                   const Cycles L2Hit_late = Cycles(10);
                   this->walker->schedule(event,this->walker->clockEdge(L2Hit_late));
		   delayedResponse = true;
		   return fault_l;
                   // End L2 latency
                   //insert_l1(vaddr,*entry,pcid);
                }else{
                   if (FullSystem) {
                    stats.l2_tlb_Misses++;
                    Fault fault = walker->start(tc, translation, req, mode);
                    if (timing || fault != NoFault) {
                        // This gets ignored in atomic mode.
                        delayedResponse = true;
                        return fault;
                    }
                    entry = lookup_l2(pageAlignedVaddr);
                    //assert(entry);
                  }else {
                    Process *p = tc->getProcessPtr();
                    const EmulationPageTable::Entry *pte =
                        p->pTable->lookup(vaddr);
                    if (!pte) {
                        return std::make_shared<PageFault>(vaddr, true, mode,
                                                           true, false);
                    } else {
                        Addr alignedVaddr = p->pTable->pageAlign(vaddr);
                        DPRINTF(TLB, "Mapping %#x to %#x\n", alignedVaddr,
                                pte->paddr);
			// It is incorrect
			/*
                        entry = insert_l2(alignedVaddr,pte->paddr,TlbEntry(
                                p->pTable->pid(), alignedVaddr, pte->paddr,
                                pte->flags & EmulationPageTable::Uncacheable,
                                pte->flags & EmulationPageTable::ReadOnly),
                                pcid);
				*/
                    }
                    DPRINTF(TLB, "Miss was serviced.\n");
                  }
                }
            }

            DPRINTF(TLB, "Entry found with paddr %#x, "
                    "doing protection checks.\n", entry->paddr);
	    // UAC
	    add_page_access(entry->paddr);
	    // End UAC
            // Do paging protection checks.
            bool inUser = m5Reg.cpl == 3 && !(flags & CPL0FlagBit);
            CR0 cr0 = tc->readMiscRegNoEffect(misc_reg::Cr0);
            bool badWrite = (!entry->writable && (inUser || cr0.wp));
            if ((inUser && !entry->user) ||
                (mode == BaseMMU::Write && badWrite)) {
                // The page must have been present to get into the TLB in
                // the first place. We'll assume the reserved bits are
                // fine even though we're not checking them.
                return std::make_shared<PageFault>(vaddr, true, mode, inUser,
                                                   false);
            }
            if (storeCheck && badWrite) {
                // This would fault if this were a write, so return a page
                // fault that reflects that happening.
                return std::make_shared<PageFault>(
                    vaddr, true, BaseMMU::Write, inUser, false);
            }

            Addr paddr = entry->paddr | (vaddr & mask(entry->logBytes));
            DPRINTF(TLB, "Translated %#x -> %#x.\n", vaddr, paddr);
            req->setPaddr(paddr);
            if (entry->uncacheable)
                req->setFlags(Request::UNCACHEABLE | Request::STRICT_ORDER);
        } else {
            //Use the address which already has segmentation applied.
            DPRINTF(TLB, "Paging disabled.\n");
            DPRINTF(TLB, "Translated %#x -> %#x.\n", vaddr, vaddr);
            req->setPaddr(vaddr);
        }
    } else {
        // Real mode
        DPRINTF(TLB, "In real mode.\n");
        DPRINTF(TLB, "Translated %#x -> %#x.\n", vaddr, vaddr);
        req->setPaddr(vaddr);
    }

    return finalizePhysical(req, tc, mode);
}

// UAC
void
TLB::print_eviction(){
	if (this->walker->curCycle() > last_cycle + 50000000){
		last_cycle = this->walker->curCycle();
		std::string delimeter = "=";
                std::string csv_path_string(csv_path);
                size_t pos = csv_path_string.find(delimeter);
                std::string csv_path_string_after= csv_path_string.substr(pos+delimeter.length());
		// L1
		// PA
                std::string file_csv_page_eviction_4kb = csv_path_string_after + "/page_eviction_l1_4kb.csv";
		if(!page_eviction_l1_4kb.empty()){
			std::ofstream file_eviction_l1_4kb(file_csv_page_eviction_4kb);
			if (file_eviction_l1_4kb.is_open()){
        			for (const auto& outerPair : page_eviction_l1_4kb){
        			    for (const auto& innerPair : outerPair.second){
        				    file_eviction_l1_4kb << "0x" << std::hex << outerPair.first << ",0x" << std::hex << innerPair.first << "," << std::dec << innerPair.second << "\n";
        
        			    }
        			}
                        }
        		file_eviction_l1_4kb.flush();
        		file_eviction_l1_4kb.close();
		}
                std::string file_csv_page_eviction_2mb = csv_path_string_after + "/page_eviction_l1_2mb.csv";
		if(!page_eviction_l1_2mb.empty()){
			std::ofstream file_eviction_l1_2mb(file_csv_page_eviction_2mb);
			if (file_eviction_l1_2mb.is_open()){
        			for (const auto& outerPair : page_eviction_l1_2mb){
        			    for (const auto& innerPair : outerPair.second){
        				    file_eviction_l1_2mb << "0x" << std::hex << outerPair.first << ",0x" << std::hex << innerPair.first << "," << std::dec << innerPair.second << "\n";
        
        			    }
        			}
                        }
        		file_eviction_l1_2mb.flush();
        		file_eviction_l1_2mb.close();
		}
		//  VA
                std::string file_csv_page_eviction_4kb_VA = csv_path_string_after + "/page_eviction_l1_4kb_VA.csv";
		if(!page_eviction_l1_4kb_VA.empty()){
			std::ofstream file_eviction_l1_4kb_VA(file_csv_page_eviction_4kb_VA);
			if (file_eviction_l1_4kb_VA.is_open()){
        		    for (auto x : page_eviction_l1_4kb_VA){
                                file_eviction_l1_4kb_VA << "0x" << std::hex << x.first << "," <<std::dec << x.second << std::endl;
			    }
                        }
        		file_eviction_l1_4kb_VA.flush();
        		file_eviction_l1_4kb_VA.close();
		}
                std::string file_csv_page_eviction_2mb_VA = csv_path_string_after + "/page_eviction_l1_2mb_VA.csv";
		if(!page_eviction_l1_2mb_VA.empty()){
			std::ofstream file_eviction_l1_2mb_VA(file_csv_page_eviction_2mb_VA);
			if (file_eviction_l1_2mb_VA.is_open()){
        		    for (auto x : page_eviction_l1_2mb_VA){
                                file_eviction_l1_2mb_VA << "0x" << std::hex << x.first << "," <<std::dec << x.second << std::endl;
			    }
                        }
        		file_eviction_l1_2mb_VA.flush();
        		file_eviction_l1_2mb_VA.close();
		}
		
		// L2
		// PA
                std::string file_csv_page_eviction_l2_4kb = csv_path_string_after + "/page_eviction_l2_4kb.csv";
		if(!page_eviction_l2_4kb.empty()){
			std::ofstream file_eviction_l2_4kb(file_csv_page_eviction_l2_4kb);
			if (file_eviction_l2_4kb.is_open()){
        			for (const auto& outerPair : page_eviction_l2_4kb){
        			    for (const auto& innerPair : outerPair.second){
        				    file_eviction_l2_4kb << "0x" << std::hex << outerPair.first << ",0x" << std::hex << innerPair.first << "," << std::dec << innerPair.second << "\n";
        
        			    }
        			}
                       }
        		file_eviction_l2_4kb.flush();
        		file_eviction_l2_4kb.close();
		}else{
			//printf("Why???\n");
		}
                std::string file_csv_page_eviction_l2_2mb = csv_path_string_after + "/page_eviction_l2_2mb.csv";
		if(!page_eviction_l2_2mb.empty()){
			std::ofstream file_eviction_l2_2mb(file_csv_page_eviction_l2_2mb);
			if (file_eviction_l2_2mb.is_open()){
        			for (const auto& outerPair : page_eviction_l2_2mb){
        			    for (const auto& innerPair : outerPair.second){
        				    file_eviction_l2_2mb << "0x" << std::hex << outerPair.first << ",0x" << std::hex << innerPair.first << "," << std::dec << innerPair.second << "\n";
        
        			    }
        			}
                        }
        		file_eviction_l2_2mb.flush();
        		file_eviction_l2_2mb.close();
		}else{
			//printf("Why???\n");
		}
		// VA
                std::string file_csv_page_eviction_l2_4kb_VA = csv_path_string_after + "/page_eviction_l2_4kb_VA.csv";
		if(!page_eviction_l2_4kb_VA.empty()){
			std::ofstream file_eviction_l2_4kb_VA(file_csv_page_eviction_l2_4kb);
			if (file_eviction_l2_4kb_VA.is_open()){
        		    for (auto x : page_eviction_l2_4kb_VA){
                                file_eviction_l2_4kb_VA << "0x" << std::hex << x.first << "," <<std::dec << x.second << std::endl;
			    }
                        }
        		file_eviction_l2_4kb_VA.flush();
        		file_eviction_l2_4kb_VA.close();
		}else{
			//printf("Why???\n");
		}
                std::string file_csv_page_eviction_l2_2mb_VA = csv_path_string_after + "/page_eviction_l2_2mb_VA.csv";
		if(!page_eviction_l2_2mb_VA.empty()){
			std::ofstream file_eviction_l2_2mb_VA(file_csv_page_eviction_l2_2mb);
			if (file_eviction_l2_2mb_VA.is_open()){
        		    for (auto x : page_eviction_l2_2mb_VA){
                                file_eviction_l2_2mb_VA << "0x" << std::hex << x.first << "," <<std::dec << x.second << std::endl;
			    }
                        }
        		file_eviction_l2_2mb_VA.flush();
        		file_eviction_l2_2mb_VA.close();
		}else{
			//printf("Why???\n");
		}
                std::string file_csv_page_eviction_l1_time = csv_path_string_after + "/page_eviction_l1_time.csv";
		if(!page_eviction_l1_time.empty()){
			std::ofstream file_eviction_l1_time(file_csv_page_eviction_l1_time);
			if (file_eviction_l1_time.is_open()){
        		    for (auto x : page_eviction_l1_time){
                                file_eviction_l1_time << "0x" << std::hex << x.first << "," <<std::dec << x.second << std::endl;
			    }
                        }
        		file_eviction_l1_time.flush();
        		file_eviction_l1_time.close();
		}
                std::string file_csv_page_access = csv_path_string_after + "/page_access.csv";
		if(!page_access.empty()){
			std::ofstream file_page_access(file_csv_page_access);
			if (file_page_access.is_open()){
        		    for (auto x : page_access){
                                file_page_access << "0x" << std::hex << x.first << "," <<std::dec << x.second << std::endl;
			    }
                        }
        		file_page_access.flush();
        		file_page_access.close();
		}
	}

}
// End UAC



Fault
TLB::translateAtomic(const RequestPtr &req, ThreadContext *tc,
    BaseMMU::Mode mode)
{
    bool delayedResponse;
    return TLB::translate(req, tc, NULL, mode, delayedResponse, false);
}

Fault
TLB::translateFunctional(const RequestPtr &req, ThreadContext *tc,
    BaseMMU::Mode mode)
{
    unsigned logBytes;
    const Addr vaddr = req->getVaddr();
    Addr addr = vaddr;
    Addr paddr = 0;
    if (FullSystem) {
        Fault fault = walker->startFunctional(tc, addr, logBytes, mode);
        if (fault != NoFault)
            return fault;
        paddr = insertBits(addr, logBytes - 1, 0, vaddr);
    } else {
        Process *process = tc->getProcessPtr();
        const auto *pte = process->pTable->lookup(vaddr);

        if (!pte && mode != BaseMMU::Execute) {
            // Check if we just need to grow the stack.
            if (process->fixupFault(vaddr)) {
                // If we did, lookup the entry for the new page.
                pte = process->pTable->lookup(vaddr);
            }
        }

        if (!pte)
            return std::make_shared<PageFault>(vaddr, true, mode, true, false);

        paddr = pte->paddr | process->pTable->pageOffset(vaddr);
    }
    DPRINTF(TLB, "Translated (functional) %#x -> %#x.\n", vaddr, paddr);
    req->setPaddr(paddr);
    return NoFault;
}

void
TLB::translateTiming(const RequestPtr &req, ThreadContext *tc,
    BaseMMU::Translation *translation, BaseMMU::Mode mode)
{
    bool delayedResponse;
    assert(translation);
    Fault fault =
        TLB::translate(req, tc, translation, mode, delayedResponse, true);
    if (!delayedResponse)
        translation->finish(fault, req, tc, mode);
    else
        translation->markDelayed();
}

Walker *
TLB::getWalker()
{
    return walker;
}

TLB::TlbStats::TlbStats(statistics::Group *parent)
  : statistics::Group(parent),
    ADD_STAT(rdAccesses, statistics::units::Count::get(),
             "TLB accesses on read requests"),
    ADD_STAT(wrAccesses, statistics::units::Count::get(),
             "TLB accesses on write requests"),
    ADD_STAT(rdMisses, statistics::units::Count::get(),
             "TLB misses on read requests"),
    ADD_STAT(wrMisses, statistics::units::Count::get(),
             "TLB misses on write requests"),
    ADD_STAT(l2_tlb_Misses, statistics::units::Count::get(), "L2 TLB misses"),
    ADD_STAT(l2_tlb_Accesses, statistics::units::Count::get(), "L2 TLB look-ups"),
    ADD_STAT(PW_latency, statistics::units::Count::get(), "PW latency"),
    ADD_STAT(PW_number, statistics::units::Count::get(), "PW Number"),
    ADD_STAT(PW_average_latency, statistics::units::Count::get(), "PW Average Latency",PW_latency / PW_number),

    ADD_STAT(L1TLB_l1_access, statistics::units::Count::get(), "L1Cache Access (L2 TLB HIT)"),
    ADD_STAT(L1TLB_l1_miss, statistics::units::Count::get(), "L1Cache Misses (L2 TLB HIT)"),
    ADD_STAT(L1TLB_l1_hit, statistics::units::Count::get(), "L1Cache Hitts (L2 TLB HIT)"),

    ADD_STAT(L2TLB_l1_access, statistics::units::Count::get(), "L1Cache Access (L2 TLB Miss)"),
    ADD_STAT(L2TLB_l1_miss, statistics::units::Count::get(), "L1Cache Misses (L2 TLB Miss)"),
    ADD_STAT(L2TLB_l1_hit, statistics::units::Count::get(), "L1Cache Hitts (L2 TLB Misses )"),


    ADD_STAT(L1TLB_l2_access, statistics::units::Count::get(), "L2Cache Access (L2 TLB HIT)"),
    ADD_STAT(L1TLB_l2_miss, statistics::units::Count::get(), "L2Cache Misses (L2 TLB HIT)"),
    ADD_STAT(L1TLB_l2_hit, statistics::units::Count::get(), "L2Cache Hitts (L2 TLB HIT)"),

    ADD_STAT(L2TLB_l2_access, statistics::units::Count::get(), "L2Cache Access (L2 TLB Miss)"),
    ADD_STAT(L2TLB_l2_miss, statistics::units::Count::get(), "L2Cache Misses (L2 TLB Miss)"),
    ADD_STAT(L2TLB_l2_hit, statistics::units::Count::get(), "L2Cache Hitts (L2 TLB Misses )"),

    ADD_STAT(ByPass_L1, statistics::units::Count::get(), "# of ByPass L1 Cache Request"),
    ADD_STAT(ByPass_L2, statistics::units::Count::get(), "# of ByPass L2 Cache Request")

{
}

void
TLB::serialize(CheckpointOut &cp) const
{
    // Only store the entries in use.
    uint32_t _size = size - freeList.size();
    SERIALIZE_SCALAR(_size);
    SERIALIZE_SCALAR(lruSeq);

    uint32_t _sizel2 = size - freeList_l2.size();
    SERIALIZE_SCALAR(_sizel2);
    SERIALIZE_SCALAR(lruSeq_l2);

    uint32_t _count = 0;
    for (unsigned set = 0; set < size; set++) {
        if (tlb[set].trieHandle != NULL)
            tlb[set].serializeSection(cp, csprintf("Entry%d", _count++));

    }
    _count = 0;
    for (unsigned set = 0; set < l2_tlb_size; set++) {
        if (l2tlb[set].trieHandle != NULL)
            l2tlb[set].serializeSection(cp, csprintf("Entry%d", _count++));
    }
}

void
TLB::unserialize(CheckpointIn &cp)
{

    // Do not allow to restore with a smaller tlb.
    uint32_t _size;
    UNSERIALIZE_SCALAR(_size);
    if (_size > size) {
        fatal("TLB size less than the one in checkpoint!");
    }

    UNSERIALIZE_SCALAR(lruSeq);
    UNSERIALIZE_SCALAR(lruSeq_l2);

    for (uint32_t x = 0; x < _size; x++) {
        TlbEntry *newEntry = freeList.front();
        freeList.pop_front();

        newEntry->unserializeSection(cp, csprintf("Entry%d", x));
        newEntry->trieHandle = trie.insert(newEntry->vaddr,
            TlbEntryTrie::MaxBits - newEntry->logBytes, newEntry);
    }
    /*
    for (uint32_t x = 0; x < l2_tlb_size; x++) {
        TlbEntry *newEntry = freeList_l2.front();
        freeList_l2.pop_front();

        newEntry->unserializeSection(cp, csprintf("Entry%d", x));
        newEntry->trieHandle = triel2.insert(newEntry->vaddr,
            TlbEntryTrie::MaxBits - newEntry->logBytes, newEntry);
    }
    */

}

TLB::DelayedL2HitEvent::DelayedL2HitEvent(
    TLB *_tlb, ThreadContext *_tc, BaseMMU::Translation *_translation,
    const RequestPtr &_req, BaseMMU::Mode _mode, Fault _fault,uint64_t _pcid,Addr _vaddr,const TlbEntry _entry)
    : tlb(_tlb), tc(_tc), translation(_translation),
    req(_req), mode(_mode), fault(_fault),pcid_l(_pcid), vaddr_l(_vaddr),entry_l(_entry), n("DelayedTranslationEvent"){
  assert(this->fault== NoFault && "Fault on DelayedTranslation.");
  this->setFlags(EventBase::AutoDelete);
}

void TLB::DelayedL2HitEvent:: process(){
    if (this->translation == NULL || this->translation == nullptr){
	return;
    }
    if(this->translation->squashed()){
	this->translation->finish(
		std::make_shared<UnimpFault>("Squashed Inst"),
		this->req,this->tc, this->mode);
        return;
    }
   bool delayedResponse;
   bool timing = true;
   //bool updateStats = false;
   //Fault fault = tlb->translate_stats(
   //	req,tc,NULL,mode,delayedResponse,
   //     timing);
   //assert(fault == NoFault && "Error in delayed Translation");
   //DPRINTF(TLB,"NAME IS:%s.\n",tlb->name());
   Fault fault = NoFault;
   DPRINTF(TLB,"Insert in L1 miss of L2:Addr:%#x.Enabel hit_l2 flags\n",this->req->getVaddr());
   //DPRINTF(TLB,"Insert in L1 miss of L2.vaddr_l:%#x.\n",vaddr_l);
   //DPRINTF(TLB,"Insert in L1 miss of L2.entry_l:%#x.\n",entry_l.vaddr);
   tlb->insert_l1(entry_l.vaddr, entry_l, pcid_l);
   fault = tlb->translate(req, tc, translation, mode, delayedResponse,timing);
   //assert(fault == NoFault && "Error in L2 Delayed Translation");
   if (tlb->name() == "system.cpu.mmu.dtb"){
       //req->set_hit_l2();
       //req->reset_miss_l2();
       tlb->incr_ByPass_L1();
   }
   if (fault == NoFault){
       assert(req->hasPaddr() && "Req has no paddr.\n");
   }

   //tlb->insert_l1(vaddr_l, entry_l, pcid_l);
   //tlb->xxx();
   this->translation->finish(fault,req,tc,mode);

 }



Port *
TLB::getTableWalkerPort()
{
    return &walker->getPort("port");
}

} // namespace X86ISA
} // namespace gem5
