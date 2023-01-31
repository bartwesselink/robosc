import { ComponentFixture, TestBed } from '@angular/core/testing';

import { ConceptsBaseComponent } from './concepts-base.component';

describe('ConceptsBaseComponent', () => {
  let component: ConceptsBaseComponent;
  let fixture: ComponentFixture<ConceptsBaseComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [ ConceptsBaseComponent ]
    })
    .compileComponents();

    fixture = TestBed.createComponent(ConceptsBaseComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
