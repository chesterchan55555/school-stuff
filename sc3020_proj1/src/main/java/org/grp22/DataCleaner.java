package org.grp22;

import java.io.*;
import java.util.*;
import java.nio.ByteBuffer;
import java.time.LocalDate;
import java.time.format.DateTimeFormatter;

public class DataCleaner {

    public static class Record {
        public int gameDate;       // encoded as yyyyMMdd (4 bytes)
        public int teamId;
        public short pts, fgPct, ftPct, fg3Pct, ast, reb;
        public byte win;

        // Constructor
        public Record(int gameDate, int teamId, short pts, short fgPct, short ftPct,
                      short fg3Pct, short ast, short reb, byte win) {
            this.gameDate = gameDate;
            this.teamId = teamId;
            this.pts = pts;
            this.fgPct = fgPct;
            this.ftPct = ftPct;
            this.fg3Pct = fg3Pct;
            this.ast = ast;
            this.reb = reb;
            this.win = win;
        }

        // No-arg constructor
        public Record() {}

        // Serialize to 22 bytes
        public byte[] toBytes() {
            ByteBuffer buffer = ByteBuffer.allocate(22);
            buffer.putInt(gameDate); // 4 bytes
            buffer.putInt(teamId);   // 4 bytes
            buffer.putShort(pts);    // 2 bytes
            buffer.putShort(fgPct);
            buffer.putShort(ftPct);
            buffer.putShort(fg3Pct);
            buffer.putShort(ast);
            buffer.putShort(reb);
            buffer.put(win);         // 1 byte
            buffer.put((byte)0);     // padding to align to 2 bytes
            return buffer.array();
        }

        // Deserialize from ByteBuffer
        public static Record fromBytes(ByteBuffer buffer) {
            Record r = new Record();
            r.gameDate = buffer.getInt();
            r.teamId = buffer.getInt();
            r.pts = buffer.getShort();
            r.fgPct = buffer.getShort();
            r.ftPct = buffer.getShort();
            r.fg3Pct = buffer.getShort();
            r.ast = buffer.getShort();
            r.reb = buffer.getShort();
            r.win = buffer.get();
            buffer.get(); // skip padding
            return r;
        }

        // Convert string date like "3/4/2022" or "12/25/2022" to int yyyyMMdd
        public static int parseDate(String dateStr) {
            if (dateStr == null || dateStr.isEmpty()) return 0;
            DateTimeFormatter formatter = DateTimeFormatter.ofPattern("d/M/yyyy");
            LocalDate date = LocalDate.parse(dateStr, formatter);
            // Encode as yyyyMMdd integer
            return date.getYear() * 10000 + date.getMonthValue() * 100 + date.getDayOfMonth();
        }

        // Decode int yyyyMMdd back to LocalDate
        public static LocalDate decodeDate(int encodedDate) {
            int year = encodedDate / 10000;
            int month = (encodedDate / 100) % 100;
            int day = encodedDate % 100;
            return LocalDate.of(year, month, day);
        }
    }

    // Clean raw text file
    public static List<Record> clean(String inputFile) throws IOException {
        List<Record> records = new ArrayList<>();
        try (BufferedReader br = new BufferedReader(new FileReader(inputFile))) {
            String line;
            boolean firstLine = true; // skip header
            while ((line = br.readLine()) != null) {
                line = line.trim();
                if (line.isEmpty()) continue;
                if (firstLine) { firstLine = false; continue; }

                String[] parts = line.split("\t", -1);
                for (int j = 0; j < parts.length; j++) parts[j] = parts[j].trim();

                int gameDate = parts.length > 0 && !parts[0].isEmpty() ? Record.parseDate(parts[0]) : 0;
                int teamId = parts.length > 1 && !parts[1].isEmpty() ? Integer.parseInt(parts[1]) : 0;
                short pts = parts.length > 2 && !parts[2].isEmpty() ? Short.parseShort(parts[2]) : 0;
                short fgPct = parts.length > 3 && !parts[3].isEmpty() ? (short)(Float.parseFloat(parts[3])*1000) : 0;
                short ftPct = parts.length > 4 && !parts[4].isEmpty() ? (short)(Float.parseFloat(parts[4])*1000) : 0;
                short fg3Pct = parts.length > 5 && !parts[5].isEmpty() ? (short)(Float.parseFloat(parts[5])*1000) : 0;
                short ast = parts.length > 6 && !parts[6].isEmpty() ? Short.parseShort(parts[6]) : 0;
                short reb = parts.length > 7 && !parts[7].isEmpty() ? Short.parseShort(parts[7]) : 0;
                byte win = parts.length > 8 && !parts[8].isEmpty() ? Byte.parseByte(parts[8]) : 0;

                records.add(new Record(gameDate, teamId, pts, fgPct, ftPct, fg3Pct, ast, reb, win));
            }
        }
        System.err.println("Total records cleaned: " + records.size());
        return records;
    }
}
