/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package javatest;

/**
 *
 * @author sa
 */
public class CacheSim {
    int cacheSize = 512*1024;
    int byteSkip=65536;
    
    int accessSize = (int)(cacheSize*1.25);
    
    public CacheSim(){
        Cache cache = new Cache();
        
        for (int j = 0; j < 10; j++) {
            for (int i = 0; i < accessSize/4; i++) {
                int address = (i * byteSkip) % accessSize;



                cache.access(address);
            }
        }
        
        
        cache.printHitRate();
        
        
//        Cache.CacheSet set = new Cache().new CacheSet();
//        for (int j = 0; j < 16; j++) {
//            for (int i = 0; i < 32; i+=4) {
//            
//                set.access(i + (j<<16));
//                set.printSet();
//                System.out.println("--------------");
//            }
//        }
        
    }
    
    
    
    class Cache{
        static final private int WORDS_PER_LINE = 8;         //note: hardcoded to 8 line per set,32 bytes per line
        static final private int LINES_PER_SET = 8;
        static final private int TAG_MASK = 0xFFFF0000;
        
        static final private int SETS_PER_CACHE = 2048;
        static final private int INDEX_MASK = 0x0000FFE0;
        static final private int INDEX_SHIFT = 5;
        
        private CacheSet[] sets;
        
        Cache(){
            sets = new CacheSet[SETS_PER_CACHE];
            for (int i = 0; i < sets.length; i++) {
                sets[i] = new CacheSet();
            }
        }
        public void access(int address){
            int index = getIndex(address);
            
            //System.out.println(index);
            
            sets[index].access(address);
        }
        public void printHitRate(){
            int hitCount=0;
            int accessCount=0;
            
            for (int i = 0; i < sets.length; i++) {
                hitCount+=sets[i].getHitCount();
                accessCount+=sets[i].getAccessCount();
            }
            
            System.out.println("hit/access="+hitCount+"/"+accessCount+", hit rate="+100.0*hitCount/accessCount+"%");
            
        }
        private int getIndex(int address){
            return (address&INDEX_MASK)>>INDEX_SHIFT;
        }
        
        
        
        
        
        public class CacheSet{
            
            final private int[] LRUTREE_UPDATE_MASK={
                (1)|(1<<1)|(1<<2),
                (1)|(1<<1)|(1<<2),
                (1)|(1<<1)|(1<<3),
                (1)|(1<<1)|(1<<3),
                (1)|(1<<4)|(1<<5),
                (1)|(1<<4)|(1<<5),
                (1)|(1<<4)|(1<<6),
                (1)|(1<<4)|(1<<6)
            };
            final private int[] LRUTREE_UPDATE_TABLE={
                (1)|(1<<1)|(1<<2),
                (1)|(1<<1)|(0<<2),
                (1)|(0<<1)|(1<<3),
                (1)|(0<<1)|(0<<3),
                (0)|(1<<4)|(1<<5),
                (0)|(1<<4)|(0<<5),
                (0)|(0<<4)|(1<<6),
                (0)|(0<<4)|(0<<6)
            };

            private int[] storedAddress;
            public int lruTree;

            private int accessCount;
            private int hitCount;


            public CacheSet(){
                storedAddress = new int[LINES_PER_SET];
                lruTree=0;
                accessCount=0;
                hitCount=0;

            }
            public void access(int address){
                accessCount++;
                int index = hitIndex(address);
                if(index == -1){        //cache miss.
                    index = getLruIndex();
                    lruTreeUpdate(index);

                    storedAddress[index] = address & TAG_MASK | 1;

                    //System.out.println("miss."+index);
                }else{                  //cache hit.
                    hitCount++;
                    lruTreeUpdate(index);
                    //System.out.println("hit."+index);
                }

            }
            public void printSet(){
                for (int i = 0; i < storedAddress.length; i++) {
                    System.out.print(i+":");
                    if(isValid(storedAddress[i])){
                        System.out.println((storedAddress[i]>>16));
                    }else{
                        System.out.println("-");
                    }
                }
            }
            public void printHitMissRate(){
                System.out.println("hit/access="+hitCount+"/"+accessCount+". hit rate="+100.0*hitCount/accessCount+"%");
            }
            public int getHitCount(){
                return hitCount;
            }
            public int getAccessCount(){
                return accessCount;
            }


            private int hitIndex(int address){
                for (int i = 0; i < storedAddress.length ; i++) {
                    if(isTagMatch(storedAddress[i], address) && isValid(storedAddress[i])){
                        return i;
                    }
                }
                return -1;

            }
            private boolean isTagMatch(int address1,int address2){
                return getTag(address1)==getTag(address2);
            }
            private boolean isValid(int address){
                return (address&1) == 1;
            }
            private int getLruIndex(){
                int bitInInterest=0;
                int index=0;

                if((lruTree & (1<<bitInInterest)) == 0){
                    bitInInterest += 1;
                    index += 0;
                }else{
                    bitInInterest += 4;
                    index += 4;
                }

                if((lruTree & (1<<bitInInterest)) == 0){
                    bitInInterest += 1;
                    index += 0;
                }else{
                    bitInInterest += 2;
                    index += 2;
                }

                if((lruTree & (1<<bitInInterest)) == 0){
                    index += 0;
                }else{
                    index += 1;
                }

                return index;
            }

            private void lruTreeUpdate(int accessedIndex){
                lruTree = (lruTree & ~LRUTREE_UPDATE_MASK[accessedIndex]) | LRUTREE_UPDATE_TABLE[accessedIndex];
            }
            private int getTag(int address){
                return address & TAG_MASK;
            }
            class CacheLine{
                private int[] data;
                public CacheLine() {
                    data = new int[WORDS_PER_LINE];
                }
                public int getWord(int offset){
                    return data[offset];
                }
                public void setWord(int offset,int word){
                    data[offset]=word;
                }
            }
        }
    }

}
