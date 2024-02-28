/*
 * Copyright (c) 2007 The Hewlett-Packard Development Company
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

#ifndef __ARCH_X86_TLB_HH__
#define __ARCH_X86_TLB_HH__

#include <list>
#include <vector>

#include "arch/generic/tlb.hh"
#include "arch/x86/pagetable.hh"
#include "base/trie.hh"
#include "mem/request.hh"
#include "params/X86TLB.hh"
#include "sim/stats.hh"
#include "string"
#include "mem/ruby/structures/CacheMemory.hh"


extern char* csv_path;


namespace gem5
{

class ThreadContext;

namespace X86ISA
{
    class Walker;

    class TLB : public BaseTLB
    {
      protected:
        friend class Walker;

        typedef std::list<TlbEntry *> EntryList;

        uint32_t configAddress;

      public:

        typedef X86TLBParams Params;
        TLB(const Params &p);

        void takeOverFrom(BaseTLB *otlb) override {}

        TlbEntry *lookup_l1(Addr va, bool update_lru = true);
        TlbEntry *lookup_l2(Addr va, bool update_lru = true);

        void setConfigAddress(uint32_t addr);
        //concatenate Page Addr and pcid
        inline Addr concAddrPcid(Addr vpn, uint64_t pcid)
        {
          return (vpn | pcid);
        }

      protected:

        EntryList::iterator lookupIt(Addr va, bool update_lru = true);

        Walker * walker;
	// UAC
	// PA
	std::unordered_map<Addr, uint64_t> page_access = {};
	std::unordered_map<Addr, uint64_t> CL_access = {};
	/*
	std::unordered_map<Addr, uint64_t> page_eviction_l1_4kb = {};
	std::unordered_map<Addr, uint64_t> page_eviction_l1_2mb = {};
	std::unordered_map<Addr, uint64_t> page_eviction_l2_4kb = {};
	std::unordered_map<Addr, uint64_t> page_eviction_l2_2mb = {};
	*/
	std::unordered_map<Addr, uint64_t> page_eviction_l1_time = {};
	// VA
	std::unordered_map<Addr, uint64_t> page_access_VA = {};
	std::unordered_map<Addr, uint64_t> page_eviction_l1_4kb_VA = {};
	std::unordered_map<Addr, uint64_t> page_eviction_l1_2mb_VA = {};
	std::unordered_map<Addr, uint64_t> page_eviction_l2_4kb_VA = {};
	std::unordered_map<Addr, uint64_t> page_eviction_l2_2mb_VA = {};
	std::unordered_map<Addr, uint64_t> page_eviction_l1_time_VA = {};
	// New
	std::unordered_map<Addr, std::unordered_map<Addr, uint64_t>> page_eviction_l1_4kb ;
	std::unordered_map<Addr, std::unordered_map<Addr, uint64_t>> page_eviction_l1_2mb ;
	std::unordered_map<Addr, std::unordered_map<Addr, uint64_t>> page_eviction_l2_4kb ;
	std::unordered_map<Addr, std::unordered_map<Addr, uint64_t>> page_eviction_l2_2mb ;
	//
	Cycles last_cycle = Cycles(0);
	// UAC End

      public:
	// UAC
	// PA
	void add_CL_access(Addr CL) {CL_access[CL]++;}
	void add_page_access(Addr address_t){ page_access[address_t]++;}
	void add_page_eviction_l1_4kb(Addr, Addr);
	void add_page_eviction_l1_2mb(Addr, Addr);
	void add_page_eviction_l2_4kb(Addr address_v, Addr address_p){page_eviction_l2_4kb[address_v][address_p] = page_eviction_l2_4kb[address_v][address_p] + 1 ;}
	void add_page_eviction_l2_2mb(Addr address_v, Addr address_p){page_eviction_l2_2mb[address_v][address_p] = page_eviction_l2_2mb[address_v][address_p] + 1 ;}
	// VA
	void add_page_access_VA(Addr address_t){ page_access[address_t]++;}
	void add_page_eviction_l1_4kb_VA(Addr address_t);
	void add_page_eviction_l1_2mb_VA(Addr address_t);
	void add_page_eviction_l2_4kb_VA(Addr address_t){page_eviction_l2_4kb_VA[address_t] = page_eviction_l2_4kb_VA[address_t] + 1 ;}
	void add_page_eviction_l2_2mb_VA(Addr address_t){page_eviction_l2_2mb_VA[address_t] = page_eviction_l2_2mb_VA[address_t] + 1 ;}

	//void print_eviction();
	// End UAC
        Walker *getWalker();

        void flushAll() override;

        void flushNonGlobal();

        void demapPage(Addr va, uint64_t asn) override;

      protected:
        uint32_t size;
        uint32_t l2_tlb_size ;
        uint32_t l2_tlb_assoc;

        uint32_t l1_way;
        std::vector<TlbEntry> tlb;
        std::vector<TlbEntry> l2tlb;

        EntryList freeList;
        EntryList freeList_l2;

        TlbEntryTrie trie;
        TlbEntryTrie triel2;
        uint64_t lruSeq;
        uint64_t lruSeq_l2;

        AddrRange m5opRange;

        struct TlbStats : public statistics::Group
        {
            TlbStats(statistics::Group *parent);

            statistics::Scalar rdAccesses;
            statistics::Scalar wrAccesses;
            statistics::Scalar rdMisses;
            statistics::Scalar wrMisses;

            statistics::Scalar l2_tlb_Accesses;
            statistics::Scalar l2_tlb_Misses;
            //
	    statistics::Scalar PW_latency;
	    statistics::Scalar PW_number;
	    statistics::Formula PW_average_latency;
	    // Cache Look up
	    statistics::Scalar L2TLB_l1_miss;
	    statistics::Scalar L2TLB_l1_hit;
	    statistics::Scalar L2TLB_l1_access;

	    statistics::Scalar L2TLB_l2_miss;
	    statistics::Scalar L2TLB_l2_hit;
	    statistics::Scalar L2TLB_l2_access;

	    statistics::Scalar L1TLB_l1_miss;
	    statistics::Scalar L1TLB_l1_hit;
	    statistics::Scalar L1TLB_l1_access;

	    statistics::Scalar L1TLB_l2_miss;
	    statistics::Scalar L1TLB_l2_hit;
	    statistics::Scalar L1TLB_l2_access;

	    statistics::Scalar ByPass_L1;
	    statistics::Scalar ByPass_L2;
        } stats;

        Fault translateInt(bool read, RequestPtr req, ThreadContext *tc);

        Fault translate(const RequestPtr &req, ThreadContext *tc,
                BaseMMU::Translation *translation, BaseMMU::Mode mode,
                bool &delayedResponse, bool timing);

      public:
	void incr_ByPass_L1() { stats.ByPass_L1++;}
	void incr_ByPass_L2() { stats.ByPass_L2++;}
	uint64_t added_cycles = 0;
	void incr_pw_latency(uint64_t delay_pw) {
		added_cycles += delay_pw;
		stats.PW_latency = added_cycles;}
	void incr_pw_number() { stats.PW_number++;}

        void evictLRU_l1();
        void evictLRU_l2();

        void evictLRU_l1(Addr);
        void evictLRU_l2(Addr);
        uint64_t
        nextSeq()
        {
            return ++lruSeq;
        }

        uint64_t
        nextSeq_l2()
        {
            return ++lruSeq_l2;
        }
        Fault translateAtomic(
            const RequestPtr &req, ThreadContext *tc,
            BaseMMU::Mode mode) override;
        Fault translateFunctional(
            const RequestPtr &req, ThreadContext *tc,
            BaseMMU::Mode mode) override;
        void translateTiming(
            const RequestPtr &req, ThreadContext *tc,
            BaseMMU::Translation *translation, BaseMMU::Mode mode) override;

        /**
         * Do post-translation physical address finalization.
         *
         * Some addresses, for example requests going to the APIC,
         * need post-translation updates. Such physical addresses are
         * remapped into a "magic" part of the physical address space
         * by this method.
         *
         * @param req Request to updated in-place.
         * @param tc Thread context that created the request.
         * @param mode Request type (read/write/execute).
         * @return A fault on failure, NoFault otherwise.
         */
        Fault finalizePhysical(const RequestPtr &req, ThreadContext *tc,
                               BaseMMU::Mode mode) const override;

        TlbEntry *insert_l1(Addr vpn, const TlbEntry &entry, uint64_t pcid);
        TlbEntry *insert_l2(Addr vpn, const TlbEntry &entry, uint64_t pcid);

        // Checkpointing
        void serialize(CheckpointOut &cp) const override;
        void unserialize(CheckpointIn &cp) override;

        /**
         * Get the table walker port. This is used for
         * migrating port connections during a CPU takeOverFrom()
         * call. For architectures that do not have a table walker,
         * NULL is returned, hence the use of a pointer rather than a
         * reference. For X86 this method will always return a valid
         * port pointer.
         *
         * @return A pointer to the walker port
         */
        Port *getTableWalkerPort() override;
        struct DelayedL2HitEvent : public Event {
 	public:
	TLB *tlb;
	ThreadContext *tc;
	BaseMMU::Translation *translation;
	RequestPtr req;
	BaseMMU::Mode mode;
	Fault fault;
	uint64_t pcid_l;
	Addr vaddr_l;
	const TlbEntry entry_l;
	std::string n;
	DelayedL2HitEvent(
		TLB *_tlb,ThreadContext *_tc,
		BaseMMU::Translation *_translation, const RequestPtr &_req,
		BaseMMU::Mode _mode, Fault _fault,uint64_t _pcid,Addr _addr,TlbEntry _entry);
	void process() override;
	const char *description() const {return "DelayedL2HitEvent";}
	const std::string name() const { return this->n;}
	};
    };

} // namespace X86ISA
} // namespace gem5

#endif // __ARCH_X86_TLB_HH__
