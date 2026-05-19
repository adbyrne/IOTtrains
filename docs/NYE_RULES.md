# NY&E Northern Lights Subdivision — Operating Rules

Selected prototype rules applied to this layout's operating sessions. Rule numbers and quoted text reference the **Standard Code of the American Railway Association, 1906 edition** (adopted April 25, 1906), the authoritative rulebook for our era. Source file: `/home/abyrne/GoogleBooks/1906_The_standard_code_of_the_American_Railwa.epub`.

Supporting reference: *Rights of Trains on Single Track* by Harry W. Forman (Railroad Gazette, 1904); *Train Rules and Train Dispatching* by H. A. Dalby (1904). Both in `/home/abyrne/GoogleBooks/`.

Cross-reference: IC 1901 rulebook (user's physical TIF/JPEG archive). The IC's A. W. Sullivan served on the ARA Train Rules Committee, so IC rules closely follow the Standard Code.

---

## Superiority of Trains (Rules 71–73)

**Rule 71**
> A train is superior to another train by right, class or direction.
> Right is conferred by train order; class and direction by time-table.
> Right is superior to class or direction.
> Direction is superior as between trains of the same class.

**Rule 72**
> Trains of the first class are superior to those of the second; trains of the second class are superior to those of the third; and so on.
> Trains in the direction specified by the time-table are superior to trains of the same class in the opposite direction.

**Rule 73**
> Extra trains are inferior to regular trains.

**NY&E application:** Southward trains are superior by direction on the Northern Lights Subdivision. This is noted in the timetable footnotes ("SOUTHWARD TRAINS ARE SUPERIOR BY DIRECTION") and inferred from timetable layout — superior trains appear on the right half with bottom-up times. At a meet between same-class trains, the inferior (northward) train takes the siding. Extras are always inferior to all scheduled trains regardless of direction; between two extras, the extra in the superior direction holds the main.

**System enforcement:** The meet order form includes a direction field for Train A. The dispatcher panel computes and displays which train takes siding based on the configured superior direction (`/manage → Layout Rules`) and whether each train is extra or scheduled. The superior direction setting is owner-configurable.

---

## Train Order Numbering (Rule 203)

**Rule 203**
> Train orders will be numbered consecutively each day, beginning with No. 1 at midnight.

**System enforcement:** The TO sequence counter resets to No. 1 at each RR-time midnight (day change detected from the fast clock). The counter also resets when the fast clock is reset to day 1.

---

## Train Order Form and Content (Rules 201, 202, 204, 206)

**Rule 201**
> For movements not provided for by time-table, train orders will be issued by authority and over the signature of the [Dispatcher]. They must contain neither information nor instructions not essential to such movements. They must be brief and clear; in the prescribed forms when applicable; and without erasure, alteration or interlineation.

**Rule 202**
> Each train order must be given in the same words to all persons or trains addressed.

**Rule 204**
> Train orders must be addressed to those who are to execute them, naming the place at which each is to receive his copy. Those for a train must be addressed to the conductor and engineman.

**Rule 206**
> Regular trains will be designated in train orders by their numbers, as "No. 10," or "2d No 10," adding engine numbers if desired. Extra trains will be designated by engine numbers, and the direction as "Extra 798, East or West."

**System enforcement:** The TO issue dialog generates order text in the prescribed form. Forms active on the layout are owner-configurable (`/manage → Active Train Order Forms`). Issued orders are sent by MQTT (QoS 2) to the addressed stations and logged in the dispatcher TO log.

---

## Standard Train Order Forms (1906 Standard Code)

The following forms are prescribed by the Standard Code for single track operations. The active subset for the NY&E is set in `/manage`.

| Form | Purpose | Single-Track? |
|------|---------|---------------|
| A | Fixing Meeting Points for Opposing Trains | Yes |
| B | Authorizing a Train to Pass or Run Ahead | Yes |
| C | Giving Right Over Another Train | Yes |
| E | Time Orders (run late / wait until) | Yes |
| F | Creating Sections | Yes |
| G | Authorizing Extra Trains | Yes |
| H | Work Extras | Yes |
| J | Holding Order | Yes |
| K | Annulling a Schedule or a Section | Yes |
| L | Annulling an Order | Yes |
| M | Annulling Part of an Order | Yes |
| P | Superseding an Order or Part of an Order | Yes |
| R | Movement Against the Current of Traffic | Double track |
| S | Section of Double or More Tracks as Single Track | Double track |
| T | Notice of New Timetable | Yes |
| U | Advance Authority to Proceed from ABS Stop Signal | Block signal |
| V | Check of Trains | Yes |
| W | Change in Clearance or Register Requirements | Yes |
| X | Slow Track Conditions | Yes |
| Y | Maintenance of Way Conditional Stop | Yes |
| Z | Relief of Flag Protection | Yes |

**NY&E default active forms (1905 era, single main track):** A, B, C, E, F, G, H, J, K, L.

Forms R and S apply to double track — not applicable to the Northern Lights Subdivision mainline. Forms U applies to ABS block signaling — deferred.

---

## Governing Principles for Meet Orders (Form A)

From *Rights of Trains on Single Track* (Forman, 1904), seven principles:

1. **Meet means meet.** Trains must wait at the designated point until the opposing train arrives (markers visible). A waiting train may pass only if the opposing train becomes 12 hours late, is annulled, a new timetable takes effect showing no such train, or a superseding order is received.
2. **Positive identification.** Regular trains by number; extras by engine number. The dispatcher adds engine numbers when needed.
3. **Orders remain in effect until fulfilled, superseded, or annulled.** One order does not supersede another unless the words "instead of" appear.
4. **Extra trains lose right at their last-named station.** They must pull into the siding there.
5. **Time-orders make trains inferior only within the time-limits named** — and confer right on other addressed trains within those limits.
6. **Trains created at initial stations** by clearance card or train order; at other stations only by train order.
7. **Orders not in proper form** must be treated only as holding orders until proper orders are issued.

**Dispatcher note (Chart 6, Forman):** When issuing a meet order, the dispatcher should indicate which train takes the siding — especially when an extra meets a regular, or when the inferior train might otherwise assume it holds the main.

---

## Rule 82 — Loss of Right and Class

> Regular trains twelve hours behind their schedule time lose both right and class, and can thereafter proceed only by train order.

**NY&E application:** A scheduled train more than 12 RR-hours late must be treated as an extra. The dispatcher must issue appropriate train orders before it can proceed.

*(Not currently enforced automatically by the system — dispatcher judgment required.)*
